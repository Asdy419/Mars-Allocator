// allocator.c - Memory-Optimized Mars Rover Allocator
//
// DESIGN OVERVIEW:
// A fault-tolerant memory allocator designed for embedded systems operating
// in high-radiation environments with thread-safe arena support.
//
// MEMORY LAYOUT:
// +--------+----------+---------+--------+
// | Header | Padding  | Payload | Footer |
// | 20B    | 20B      | N bytes | 12B    |
// +--------+----------+---------+--------+
// ^                   ^
// offset 0            offset 40 (40-byte aligned)
//
// KEY FEATURES:
// 1. Segregated free lists for O(1) common-case allocation
// 2. Thread-safe heap arenas for concurrent access
// 3. Comprehensive status codes for all operations
// 4. Header recovery from footer after corruption
// 5. Quarantine system for damaged blocks
//
// ALLOCATION STRATEGY: SEGREGATED FREE LISTS WITH BEST-FIT
// Size classes: 40, 80, 120, 200, 320, 520, 840, 1360, large
// - O(1) class selection based on requested size
// - Best-fit search within each class (minimizes internal fragmentation)
// - Early exit on perfect fit for O(1) best case
// - Falls back to larger classes if current class is empty
// - Reduces fragmentation by grouping similar sizes
//
// MEMORY OPTIMIZATIONS:
// - Packed structures eliminate padding
// - CRC16 instead of CRC32 (2 bytes, 99.998% detection)
// - Table-less CRC computation (saves 1KB)
// - 32-bit offsets instead of 64-bit pointers
// - Total overhead: 32 bytes/block (was 72)

#include "allocator.h"
#include <string.h>
#include <stdio.h>
#include <threads.h>

// Configuration.
#define ALIGN 40
#define HEADER_MAGIC 0xC0FEu
#define FOOTER_MAGIC 0xBEEFu
#define MIN_PAYLOAD 8
#define NULL_OFFSET 0xFFFFFFFFu

#define ALIGN_UP(sz) (((sz) + ALIGN - 1) / ALIGN * ALIGN)

// Segregated free list size classes.
// Dynamically computed per-arena based on heap size.
#define NUM_SIZE_CLASSES 9
#define MIN_CLASS_PAYLOAD 40  // Smallest class payload size

// CRC16-CCITT table-less computation.
// Polynomial: 0x1021, Init: 0xFFFF.
static uint16_t compute_crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

// Block Header (20 bytes packed).
typedef struct __attribute__((packed)) {
  uint16_t magic;          // Magic number for validation
  uint16_t checksum;       // CRC16 of header
  uint32_t size;           // Total block size including header/footer
  uint32_t payload_size;   // User-requested size
  uint16_t payload_crc;    // CRC16 of payload data
  uint8_t allocated;       // 1 if allocated, 0 if free
  uint8_t size_class;      // Index into size class array
  uint32_t next_free_off;  // Offset to next free block in same class
} Header;

#define HEADER_SIZE sizeof(Header)

// Block Footer (12 bytes packed).
typedef struct __attribute__((packed)) {
  uint16_t magic;          // Magic number for validation
  uint16_t checksum;       // CRC16 of footer
  uint32_t size;           // Mirrors header size for backward traversal
  uint16_t payload_crc;    // Mirrors header for recovery
  uint8_t allocated;       // Mirrors header for recovery
  uint8_t pad;             // Padding for alignment
} Footer;

#define FOOTER_SIZE sizeof(Footer)
#define MIN_BLOCK_SIZE ALIGN_UP(ALIGN + MIN_PAYLOAD + FOOTER_SIZE)

// Quarantine entry for corrupted regions.
typedef struct __attribute__((packed)) {
  uint32_t start;
  uint32_t end;
} QuarantineEntry;

#define MAX_QUARANTINE 16

// Per-arena state with mutex for thread safety.
typedef struct {
  uint8_t* heap_start;
  uint32_t heap_size;
  uint32_t size_classes[NUM_SIZE_CLASSES];  // Dynamic class boundaries
  uint32_t free_lists[NUM_SIZE_CLASSES];    // Head of each segregated list
  QuarantineEntry quarantine[MAX_QUARANTINE];
  uint8_t quarantine_count;
  uint8_t initialized;
  mtx_t lock;  // Per-arena lock for thread safety
} Arena;

// Global state.
static Arena g_arenas[MM_ARENA_MAX];
static uint8_t g_num_arenas = 0;
static mtx_t g_init_lock;
static once_flag g_init_once = ONCE_FLAG_INIT;

// Thread-local arena selection.
static thread_local ArenaId tls_current_arena = 0;

static uint8_t g_unused_pattern[5];
static uint8_t g_pattern_captured = 0;

// Status code strings.
static const char* k_status_strings[] = {
  "OK",
  "Corrupt block found",
  "Quarantined regions exist",
  "Free list inconsistent",
  "Header/footer mismatch",
  "Invalid pointer",
  "Block not allocated",
  "Checksum verification failed",
  "Out of memory",
  "Invalid size",
  "Heap not initialized",
  "Lock acquisition failed"
};

const char* mm_status_str(MmStatus status) {
  if (status <= MM_ERR_LOCK_FAILED) {
    return k_status_strings[status];
  }
  return "Unknown error";
}

// Initialize global mutex once.
static void init_global_lock(void) {
  mtx_init(&g_init_lock, mtx_plain);
}

// Lock helpers.
static int arena_lock(Arena* a) {
  return mtx_lock(&a->lock) == thrd_success;
}

static void arena_unlock(Arena* a) {
  mtx_unlock(&a->lock);
}

// Get arena by ID.
static Arena* get_arena_by_id(ArenaId id) {
  if (id >= g_num_arenas) {
    return NULL;
  }
  return &g_arenas[id];
}

// Get current thread's arena.
static Arena* get_arena(void) {
  return get_arena_by_id(tls_current_arena);
}

// Find which arena a pointer belongs to (lock-free lookup).
static int find_arena_for_ptr(void* ptr) {
  if (!ptr) {
    return -1;
  }
  uint8_t* p = (uint8_t*)ptr;
  for (uint8_t i = 0; i < g_num_arenas; i++) {
    Arena* a = &g_arenas[i];
    if (a->initialized && p >= a->heap_start &&
        p < a->heap_start + a->heap_size) {
      return i;
    }
  }
  return -1;
}

// Check if arena is ready.
static int arena_ready(Arena* a) {
  return a && a->initialized && a->heap_start && a->heap_size > 0;
}

// Check if range is within arena heap.
static int range_in_heap(Arena* a, const void* p, size_t len) {
  if (!arena_ready(a) || len == 0) {
    return 0;
  }
  const uint8_t* s = (const uint8_t*)p;
  const uint8_t* e = s + len;
  return s >= a->heap_start && e <= a->heap_start + a->heap_size && s < e;
}

// Get size class index for a payload size (arena-specific).
static uint8_t get_size_class(Arena* a, uint32_t payload_size) {
  for (uint8_t i = 0; i < NUM_SIZE_CLASSES - 1; i++) {
    if (payload_size <= a->size_classes[i]) {
      return i;
    }
  }
  return NUM_SIZE_CLASSES - 1;  // Large class
}

// Initialize size classes for an arena using geometric progression.
// Classes scale from MIN_CLASS_PAYLOAD to heap_size/4.
static void init_size_classes(Arena* a) {
  uint32_t min_payload = MIN_CLASS_PAYLOAD;
  uint32_t max_payload = a->heap_size / 4;

  if (max_payload < min_payload * 2) {
    max_payload = min_payload * 2;
  }

  // Compute geometric ratio to span from min to max
  // across NUM_SIZE_CLASSES-1 classes.
  // We want: min * ratio^(N-2) = max, so ratio = (max/min)^(1/(N-2))
  // Approximate using iterative method to avoid libm dependency.
  double base = (double)max_payload / min_payload;
  int steps = NUM_SIZE_CLASSES - 2;  // Number of ratio multiplications

  // Approximate nth root: start with estimate and refine.
  double ratio = 2.0;  // Initial guess
  if (steps > 0) {
    // Newton-Raphson for x^n = base: x = x - (x^n - base)/(n * x^(n-1))
    // Simplified: x = ((n-1)*x + base/x^(n-1)) / n
    for (int iter = 0; iter < 20; iter++) {
      double xn = 1.0;
      for (int j = 0; j < steps; j++) {
        xn *= ratio;  // ratio^steps
      }
      double xn1 = xn / ratio;  // ratio^(steps-1)
      ratio = ((steps - 1) * ratio + base / xn1) / steps;
    }
  }

  // Clamp ratio to reasonable bounds.
  if (ratio < 1.3) {
    ratio = 1.3;
  }
  if (ratio > 3.0) {
    ratio = 3.0;
  }

  // Generate class boundaries.
  double current = min_payload;
  for (int i = 0; i < NUM_SIZE_CLASSES - 1; i++) {
    a->size_classes[i] = ALIGN_UP((uint32_t)current);
    // Ensure strictly increasing with minimum gap.
    if (i > 0 && a->size_classes[i] <= a->size_classes[i-1]) {
      a->size_classes[i] = a->size_classes[i-1] + ALIGN;
    }
    current *= ratio;
  }
  a->size_classes[NUM_SIZE_CLASSES - 1] = 0xFFFFFFFF;  // Unlimited
}

// Quarantine functions.
static int is_quarantined(Arena* a, uint32_t off, uint32_t len) {
  uint32_t end = off + len;
  for (uint8_t i = 0; i < a->quarantine_count; i++) {
    if (!(end <= a->quarantine[i].start || off >= a->quarantine[i].end)) {
      return 1;
    }
  }
  return 0;
}

static void quarantine_add(Arena* a, uint32_t off, uint32_t len) {
  if (!arena_ready(a) || len == 0) {
    return;
  }
  uint32_t s = (off / ALIGN) * ALIGN;
  uint32_t e = ALIGN_UP(off + len);
  if (e > a->heap_size) {
    e = a->heap_size;
  }

  // Try to merge with existing.
  for (uint8_t i = 0; i < a->quarantine_count; i++) {
    if (!(e < a->quarantine[i].start || s > a->quarantine[i].end)) {
      if (s < a->quarantine[i].start) {
        a->quarantine[i].start = s;
      }
      if (e > a->quarantine[i].end) {
        a->quarantine[i].end = e;
      }
      return;
    }
  }

  // Add new entry.
  if (a->quarantine_count < MAX_QUARANTINE) {
    a->quarantine[a->quarantine_count].start = s;
    a->quarantine[a->quarantine_count].end = e;
    a->quarantine_count++;
  }
}

// Offset/pointer conversion.
static Header* offset_to_header(Arena* a, uint32_t off) {
  if (off == NULL_OFFSET || off >= a->heap_size) {
    return NULL;
  }
  return (Header*)(a->heap_start + off);
}

static uint32_t header_to_offset(Arena* a, Header* h) {
  if (!h) {
    return NULL_OFFSET;
  }
  return (uint8_t*)h - a->heap_start;
}

// Payload access.
static uint8_t* get_payload(Arena* a, Header* h) {
  uint32_t hdr_off = (uint8_t*)h - a->heap_start;
  return a->heap_start + hdr_off + ALIGN;
}

// Footer access.
static Footer* get_footer(Arena* a, Header* h) {
  if (!h || h->size < MIN_BLOCK_SIZE) {
    return NULL;
  }
  uint8_t* f = (uint8_t*)h + h->size - FOOTER_SIZE;
  return range_in_heap(a, f, FOOTER_SIZE) ? (Footer*)f : NULL;
}

// Payload capacity.
static uint32_t get_payloadCapacity(Header* h) {
  if (!h || h->size < ALIGN + FOOTER_SIZE) {
    return 0;
  }
  return h->size - ALIGN - FOOTER_SIZE;
}

// Checksum computation.
static uint16_t compute_header_crc(const Header* h) {
  uint8_t buf[HEADER_SIZE];
  memcpy(buf, h, HEADER_SIZE);
  buf[2] = buf[3] = 0;  // Zero checksum field
  return compute_crc16(buf, HEADER_SIZE);
}

static uint16_t compute_footer_crc(const Footer* f) {
  uint8_t buf[FOOTER_SIZE];
  memcpy(buf, f, FOOTER_SIZE);
  buf[2] = buf[3] = 0;
  return compute_crc16(buf, FOOTER_SIZE);
}

static uint16_t compute_payload_crc(Arena* a, Header* h) {
  if (!h || h->payload_size == 0) {
    return 0;
  }
  return compute_crc16(get_payload(a, h), h->payload_size);
}

// Block finalization.
static void refresh_header(Header* h) {
  h->checksum = compute_header_crc(h);
}

static void refresh_footer(Arena* a, Header* h) {
  Footer* f = get_footer(a, h);
  if (!f) {
    return;
  }
  f->magic = FOOTER_MAGIC;
  f->size = h->size;
  f->payload_crc = h->payload_crc;
  f->allocated = h->allocated;
  f->pad = 0;
  f->checksum = compute_footer_crc(f);
}

// Finalize a block by updating both header and footer CRCs.
// Called after any modification to block metadata.
static void finalize_block(Arena* a, Header* h) {
  refresh_header(h);
  refresh_footer(a, h);
}

// ============================================================================
// Block Validation Functions
// Three-tier validation: header-only, footer-only, and full block validation.
// Each level provides increasing confidence in block integrity.
// ============================================================================

// Validate header structure and CRC.
// Checks: magic number, size constraints, allocation flag, bounds, checksum.
// Returns 1 if valid, 0 if any check fails.
static int header_valid(Arena* a, const Header* h) {
  // Check header is within heap bounds.
  if (!range_in_heap(a, h, HEADER_SIZE)) {
    return 0;
  }
  // Verify magic number.
  if (h->magic != HEADER_MAGIC) {
    return 0;
  }
  // Check size is valid (minimum, maximum, aligned).
  if (h->size < MIN_BLOCK_SIZE || h->size > a->heap_size ||
      h->size % ALIGN != 0) {
    return 0;
  }
  // Allocation flag must be 0 or 1.
  if (h->allocated > 1) {
    return 0;
  }
  // Check block doesn't extend past heap end.
  uint32_t off = (const uint8_t*)h - a->heap_start;
  if (off + h->size > a->heap_size) {
    return 0;
  }
  // Payload size must fit within block capacity.
  if (h->payload_size > get_payloadCapacity((Header*)h)) {
    return 0;
  }
  // Verify CRC matches computed value.
  return h->checksum == compute_header_crc(h);
}

// Validate footer structure and CRC.
// Similar checks to header but for the block's footer.
// Returns 1 if valid, 0 if any check fails.
static int footer_valid(Arena* a, const Footer* f) {
  // Check footer is within heap bounds.
  if (!range_in_heap(a, f, FOOTER_SIZE)) {
    return 0;
  }
  // Verify magic number.
  if (f->magic != FOOTER_MAGIC) {
    return 0;
  }
  // Allocation flag must be 0 or 1.
  if (f->allocated > 1) {
    return 0;
  }
  // Check size is valid.
  if (f->size < MIN_BLOCK_SIZE || f->size > a->heap_size ||
      f->size % ALIGN != 0) {
    return 0;
  }
  // Verify CRC matches computed value.
  return f->checksum == compute_footer_crc(f);
}

// Full block validation: verify both header and footer, plus consistency.
// This is the strongest validation - both ends must be intact and agree.
// Returns 1 if fully valid, 0 otherwise.
static int block_fully_valid(Arena* a, Header* h) {
  // First validate header.
  if (!header_valid(a, h)) {
    return 0;
  }
  // Get and validate footer.
  Footer* f = get_footer(a, h);
  if (!f || !footer_valid(a, f)) {
    return 0;
  }
  // Verify header and footer agree on critical fields.
  if (h->size != f->size || h->payload_crc != f->payload_crc ||
      h->allocated != f->allocated) {
    return 0;
  }
  return 1;
}

// ============================================================================
// Block Recovery Functions
// Bidirectional redundancy: reconstruct corrupted header from footer
// or vice versa.
// Key to radiation tolerance - single-bit errors can be recovered.
// ============================================================================

// Attempt to recover a corrupted header using a valid footer.
// Requires: header has valid size field (to locate footer).
// Returns 1 on success, 0 if recovery not possible.
static int try_recover_header_from_footer(Arena* a, Header* h) {
  if (!range_in_heap(a, h, HEADER_SIZE)) {
    return 0;
  }
  if (h->size < MIN_BLOCK_SIZE || h->size > a->heap_size ||
      h->size % ALIGN != 0) {
    return 0;
  }
  uint32_t off = (uint8_t*)h - a->heap_start;
  if (off + h->size > a->heap_size) {
    return 0;
  }

  Footer* f = get_footer(a, h);
  if (!f || !footer_valid(a, f) || f->size != h->size) {
    return 0;
  }

  // Footer is valid, reconstruct header.
  h->magic = HEADER_MAGIC;
  h->payload_crc = f->payload_crc;
  h->allocated = f->allocated;
  if (h->allocated && h->payload_size > get_payloadCapacity(h)) {
    h->payload_size = get_payloadCapacity(h);
  }
  refresh_header(h);
  return 1;
}

// Recovery: reconstruct footer from valid header.
static int try_recover_footer_from_header(Arena* a, Header* h) {
  if (!header_valid(a, h)) {
    return 0;
  }

  Footer* f = get_footer(a, h);
  if (!f || !range_in_heap(a, f, FOOTER_SIZE)) {
    return 0;
  }

  // Header is valid, reconstruct footer.
  f->magic = FOOTER_MAGIC;
  f->size = h->size;
  f->payload_crc = h->payload_crc;
  f->allocated = h->allocated;
  f->pad = 0;
  f->checksum = compute_footer_crc(f);
  return 1;
}

static int validate_block(Arena* a, Header* h) {
  // Fast path: both header and footer valid.
  if (block_fully_valid(a, h)) {
    return 1;
  }

  // Try recover header from footer.
  if (try_recover_header_from_footer(a, h) && block_fully_valid(a, h)) {
    return 1;
  }

  // Try recover footer from header.
  if (try_recover_footer_from_header(a, h) && block_fully_valid(a, h)) {
    return 1;
  }

  return 0;
}

static int validate_with_payload(Arena* a, Header* h) {
  if (!validate_block(a, h)) {
    return 0;
  }
  if (h->allocated && h->payload_size > 0) {
    if (h->payload_crc != compute_payload_crc(a, h)) {
      return 0;
    }
  }
  return 1;
}

// Quarantine a block.
static void quarantine_block(Arena* a, Header* h) {
  if (!range_in_heap(a, h, HEADER_SIZE)) {
    return;
  }
  uint32_t off = (uint8_t*)h - a->heap_start;
  uint32_t len = ALIGN;
  if (h->magic == HEADER_MAGIC && h->size >= MIN_BLOCK_SIZE &&
      h->size <= a->heap_size && h->size % ALIGN == 0) {
    len = h->size;
  }
  quarantine_add(a, off, len);
}

// Pattern functions.
static void fill_pattern(Arena* a, uint8_t* p, uint32_t len) {
  if (!g_pattern_captured || !len) {
    return;
  }
  uint32_t base = p - a->heap_start;
  for (uint32_t i = 0; i < len; i++) {
    p[i] = g_unused_pattern[(base + i) % 5];
  }
}

static int matches_pattern(Arena* a, const uint8_t* p, uint32_t len) {
  if (!g_pattern_captured || len < 5) {
    return 0;
  }
  uint32_t base = p - a->heap_start;
  for (uint32_t i = 0; i < len; i++) {
    if (p[i] != g_unused_pattern[(base + i) % 5]) {
      return 0;
    }
  }
  return 1;
}

static int has_partial_pattern(Arena* a, Header* h) {
  if (!h || !h->allocated || h->payload_size < 10 || !g_pattern_captured) {
    return 0;
  }
  const uint8_t* p = get_payload(a, h);
  uint32_t base = p - a->heap_start;

  // Check if starts with pattern (would indicate unwritten).
  int starts = 1;
  for (int j = 0; j < 5 && starts; j++) {
    if (p[j] != g_unused_pattern[(base + j) % 5]) {
      starts = 0;
    }
  }
  if (starts) {
    return 0;
  }

  // Look for pattern in middle (brownout indicator).
  for (uint32_t i = 5; i + 5 <= h->payload_size; i++) {
    int match = 1;
    for (int j = 0; j < 5 && match; j++) {
      if (p[i + j] != g_unused_pattern[(base + i + j) % 5]) {
        match = 0;
      }
    }
    if (match) {
      return 1;
    }
  }
  return 0;
}

// Free list management with segregated lists.
static void remove_from_free_list(Arena* a, Header* h, uint8_t class_idx);
static void rebuild_free_lists(Arena* a);

static void add_to_free_list(Arena* a, Header* h) {
  if (!arena_ready(a) || !validate_block(a, h)) {
    quarantine_block(a, h);
    return;
  }

  h->allocated = 0;
  h->payload_size = 0;
  h->payload_crc = 0;

  // Determine size class.
  uint32_t capacity = get_payloadCapacity(h);
  uint8_t class_idx = get_size_class(a, capacity);
  h->size_class = class_idx;

  // Insert at head of appropriate list (LIFO for locality).
  h->next_free_off = a->free_lists[class_idx];
  a->free_lists[class_idx] = header_to_offset(a, h);
  finalize_block(a, h);
}

static void remove_from_free_list(Arena* a, Header* h, uint8_t class_idx) {
  uint32_t target_off = header_to_offset(a, h);
  uint32_t prev_off = NULL_OFFSET;
  uint32_t cur_off = a->free_lists[class_idx];

  while (cur_off != NULL_OFFSET) {
    if (cur_off == target_off) {
      if (prev_off == NULL_OFFSET) {
        a->free_lists[class_idx] = h->next_free_off;
      } else {
        Header* prev = offset_to_header(a, prev_off);
        if (prev && validate_block(a, prev)) {
          prev->next_free_off = h->next_free_off;
          finalize_block(a, prev);
        }
      }
      return;
    }
    Header* cur = offset_to_header(a, cur_off);
    if (!cur || !validate_block(a, cur)) {
      break;
    }
    prev_off = cur_off;
    cur_off = cur->next_free_off;
  }
}

// Coalesce (merge) adjacent free blocks to reduce fragmentation.
// Attempts to merge with both the next block (forward coalescing)
// and the previous block (backward coalescing using footer).
// Returns pointer to the (possibly merged) block.
static Header* coalesce_block(Arena* a, Header* h) {
  // Validate parameters.
  if (!arena_ready(a) || !h) {
    return h;
  }

  uint32_t h_off = header_to_offset(a, h);

  // FORWARD COALESCING: Try to merge with the next block.
  // If the next block is free, absorb it into this block.
  uint32_t next_off = h_off + h->size;
  if (next_off + MIN_BLOCK_SIZE <= a->heap_size) {
    Header* next = offset_to_header(a, next_off);
    if (next && validate_block(a, next) && !next->allocated) {
      // Next block is free - remove it from free list and merge.
      remove_from_free_list(a, next, next->size_class);
      h->size += next->size;
      finalize_block(a, h);
    }
  }

  // BACKWARD COALESCING: Try to merge with the previous block.
  // Use the footer of the previous block to find its header.
  if (h_off >= MIN_BLOCK_SIZE) {
    // Previous block's footer is just before our header.
    Footer* prev_footer = (Footer*)(a->heap_start + h_off - FOOTER_SIZE);
    if (range_in_heap(a, prev_footer, FOOTER_SIZE) &&
        footer_valid(a, prev_footer)) {
      // Calculate previous block's header offset from footer size.
      uint32_t prev_off = h_off - prev_footer->size;
      Header* prev = offset_to_header(a, prev_off);
      if (prev && validate_block(a, prev) && !prev->allocated) {
        // Previous block is free - merge both blocks into previous.
        remove_from_free_list(a, prev, prev->size_class);
        remove_from_free_list(a, h, h->size_class);
        prev->size += h->size;
        finalize_block(a, prev);
        return prev;  // Return merged block (previous absorbs current)
      }
    }
  }

  return h;  // Return original block (no merge occurred)
}

// Rebuild all free lists by scanning the entire heap.
// Called after detecting corruption to restore list consistency.
// Quarantines any invalid blocks found during the scan.
static void rebuild_free_lists(Arena* a) {
  // Verify arena is ready.
  if (!arena_ready(a)) {
    return;
  }

  // Clear all size class lists.
  for (int i = 0; i < NUM_SIZE_CLASSES; i++) {
    a->free_lists[i] = NULL_OFFSET;
  }

  // Scan entire heap and rebuild free lists.
  for (uint32_t off = 0; off + MIN_BLOCK_SIZE <= a->heap_size;) {
    // Skip quarantined regions.
    if (is_quarantined(a, off, HEADER_SIZE)) {
      off += ALIGN;
      continue;
    }

    Header* h = (Header*)(a->heap_start + off);
    if (!validate_block(a, h)) {
      quarantine_add(a, off, ALIGN);
      off += ALIGN;
      continue;
    }
    // If block is free, add to appropriate size class list.
    if (!h->allocated) {
      uint8_t class_idx = get_size_class(a, get_payloadCapacity(h));
      h->size_class = class_idx;
      h->next_free_off = a->free_lists[class_idx];
      a->free_lists[class_idx] = off;
      finalize_block(a, h);
    }
    // Move to next block.
    off += h->size;
  }
}

// Find the header for a given payload pointer.
// Validates alignment, checks quarantine, and verifies block integrity.
// Returns NULL if pointer is invalid, quarantined, or block is corrupted.
static Header* find_header(Arena* a, void* ptr) {
  // Validate input parameters.
  if (!arena_ready(a) || !ptr) {
    return NULL;
  }
  uint8_t* p = (uint8_t*)ptr;

  // Check pointer is within heap bounds.
  if (!range_in_heap(a, p, 1)) {
    return NULL;
  }

  // Verify pointer alignment (must be ALIGN-byte aligned).
  uint32_t p_off = p - a->heap_start;
  if (p_off % ALIGN != 0 || p_off < ALIGN) {
    return NULL;
  }

  // Calculate header offset (header is ALIGN bytes before payload).
  uint32_t h_off = p_off - ALIGN;

  // Check if this region is quarantined.
  if (is_quarantined(a, h_off, HEADER_SIZE)) {
    return NULL;
  }

  // Validate the block.
  Header* h = (Header*)(a->heap_start + h_off);
  if (!validate_block(a, h)) {
    // If magic is valid but block is corrupt, quarantine it.
    if (h->magic == HEADER_MAGIC) {
      quarantine_block(a, h);
    }
    return NULL;
  }

  // Verify block is allocated and payload matches.
  if (!h->allocated || get_payload(a, h) != p) {
    return NULL;
  }
  return h;
}

// ============================================================================
// Public API Implementation
// ============================================================================

// Initialize allocator with a single arena.
// Wrapper around mm_arena_init for simple single-threaded use.
MmStatus mm_init(uint8_t* heap, size_t heap_size) {
  return mm_arena_init(heap, heap_size, 1);
}

// Initialize allocator with multiple arenas for thread isolation.
// Divides heap into num_arenas equal parts, each with its own lock.
// Returns MM_OK on success, error code on failure.
MmStatus mm_arena_init(uint8_t* heap, size_t heap_size, uint8_t num_arenas) {
  // Validate parameters.
  if (!heap || heap_size < MIN_BLOCK_SIZE) {
    return MM_ERR_INVALID_SIZE;
  }
  if (num_arenas == 0 || num_arenas > MM_ARENA_MAX) {
    return MM_ERR_INVALID_SIZE;
  }

  // Initialize global lock (once).
  call_once(&g_init_once, init_global_lock);
  mtx_lock(&g_init_lock);

  // Capture 5-byte pattern.
  for (int i = 0; i < 5; i++) {
    g_unused_pattern[i] = heap[i];
  }
  g_pattern_captured = 1;

  size_t arena_size = (heap_size / num_arenas / ALIGN) * ALIGN;
  if (arena_size < MIN_BLOCK_SIZE) {
    mtx_unlock(&g_init_lock);
    return MM_ERR_INVALID_SIZE;
  }

  for (uint8_t i = 0; i < num_arenas; i++) {
    Arena* a = &g_arenas[i];

    // Destroy old mutex if reinitializing.
    if (a->initialized) {
      mtx_destroy(&a->lock);
    }

    a->heap_start = heap + i * arena_size;
    a->heap_size = arena_size;
    a->quarantine_count = 0;
    a->initialized = 1;  // Set BEFORE finalize_block (needed by range_in_heap)

    // Initialize dynamic size classes based on arena size.
    init_size_classes(a);

    // Initialize mutex.
    mtx_init(&a->lock, mtx_plain);

    // Initialize all free lists to empty.
    for (int j = 0; j < NUM_SIZE_CLASSES; j++) {
      a->free_lists[j] = NULL_OFFSET;
    }

    // Create initial free block.
    Header* h = (Header*)a->heap_start;
    h->magic = HEADER_MAGIC;
    h->size = a->heap_size;
    h->payload_size = 0;
    h->payload_crc = 0;
    h->allocated = 0;
    h->size_class = NUM_SIZE_CLASSES - 1;  // Large class
    h->next_free_off = NULL_OFFSET;
    finalize_block(a, h);

    a->free_lists[NUM_SIZE_CLASSES - 1] = 0;
  }

  g_num_arenas = num_arenas;
  tls_current_arena = 0;

  mtx_unlock(&g_init_lock);
  return MM_OK;
}

void mm_arena_select(ArenaId arena) {
  if (arena < g_num_arenas) {
    tls_current_arena = arena;
  }
}

ArenaId mm_arena_current(void) {
  return tls_current_arena;
}

// Allocates a block of at least 'size' bytes from the current arena.
// Returns pointer to usable memory, or NULL on failure.
// Uses best-fit allocation within segregated size classes.
void* mm_malloc(size_t size) {
  // Get current thread's arena.
  Arena* a = get_arena();

  // Validate request parameters.
  if (!arena_ready(a) || size == 0 || size > 0xFFFFFFFF) {
    return NULL;
  }

  // Acquire arena lock for thread-safe access.
  if (!arena_lock(a)) {
    return NULL;
  }

  // Calculate total block size needed (header + payload + footer).
  uint32_t needed = ALIGN_UP(ALIGN + (uint32_t)size + FOOTER_SIZE);
  if (needed < MIN_BLOCK_SIZE) {
    needed = MIN_BLOCK_SIZE;
  }

  // Determine starting size class based on requested size.
  uint8_t start_class = get_size_class(a, (uint32_t)size);

  // Search from appropriate class upward until we find a fit.
  for (uint8_t class_idx = start_class; class_idx < NUM_SIZE_CLASSES;
       class_idx++) {
    // Best-fit search within this size class.
    // Track the smallest block that fits the request.
    uint32_t best_off = NULL_OFFSET;
    uint32_t best_prev_off = NULL_OFFSET;
    uint32_t best_size = 0xFFFFFFFF;

    // Walk the free list for this size class.
    uint32_t cur_off = a->free_lists[class_idx];
    uint32_t prev_off = NULL_OFFSET;

    while (cur_off != NULL_OFFSET) {
      // Convert offset to header pointer.
      Header* cur = offset_to_header(a, cur_off);

      // Validate block integrity; quarantine if corrupted.
      if (!cur || !validate_block(a, cur)) {
        if (cur) {
          quarantine_block(a, cur);
        }
        // Rebuild lists after corruption detected.
        rebuild_free_lists(a);
        arena_unlock(a);
        return mm_malloc(size);  // Retry allocation
      }

      // Skip quarantined blocks and remove from list.
      if (is_quarantined(a, cur_off, cur->size)) {
        // Unlink quarantined block from free list.
        if (prev_off == NULL_OFFSET) {
          a->free_lists[class_idx] = cur->next_free_off;
        } else {
          Header* prev = offset_to_header(a, prev_off);
          if (prev) {
            prev->next_free_off = cur->next_free_off;
            finalize_block(a, prev);
          }
        }
        cur_off = cur->next_free_off;
        continue;
      }

      // Check if this block fits and is better than current best.
      if (cur->size >= needed && cur->size < best_size) {
        best_off = cur_off;
        best_prev_off = prev_off;
        best_size = cur->size;

        // Perfect fit found - no need to search further.
        if (cur->size == needed) {
          break;
        }
      }

      prev_off = cur_off;
      cur_off = cur->next_free_off;
    }

    // If we found a suitable block in this class, use it.
    if (best_off != NULL_OFFSET) {
      Header* best = offset_to_header(a, best_off);
      if (!best) {
        continue;
      }

      uint32_t rem = best->size - needed;

      // Remove from current list.
      if (best_prev_off == NULL_OFFSET) {
        a->free_lists[class_idx] = best->next_free_off;
      } else {
        Header* prev = offset_to_header(a, best_prev_off);
        if (prev) {
          prev->next_free_off = best->next_free_off;
          finalize_block(a, prev);
        }
      }

      // Split if remainder is large enough.
      if (rem >= MIN_BLOCK_SIZE) {
        Header* r = (Header*)((uint8_t*)best + needed);
        r->magic = HEADER_MAGIC;
        r->size = rem;
        r->payload_size = 0;
        r->payload_crc = 0;
        r->allocated = 0;
        r->size_class = get_size_class(a, get_payloadCapacity(r));
        r->next_free_off = a->free_lists[r->size_class];
        finalize_block(a, r);

        if (validate_block(a, r)) {
          a->free_lists[r->size_class] = header_to_offset(a, r);
          best->size = needed;
        } else {
          quarantine_block(a, r);
        }
      }

      // Mark as allocated.
      best->allocated = 1;
      best->payload_size = (uint32_t)size;
      best->size_class = class_idx;
      best->next_free_off = NULL_OFFSET;

      uint8_t* payload = get_payload(a, best);
      fill_pattern(a, payload, (uint32_t)size);
      best->payload_crc = compute_payload_crc(a, best);
      finalize_block(a, best);

      if (!validate_block(a, best)) {
        quarantine_block(a, best);
        continue;  // Try next class
      }
      arena_unlock(a);
      return payload;
    }
  }

  arena_unlock(a);
  return NULL;  // No suitable block found
}

void* mm_malloc_from(ArenaId arena, size_t size) {
  Arena* a = get_arena_by_id(arena);
  if (!arena_ready(a) || size == 0 || size > 0xFFFFFFFF) {
    return NULL;
  }

  if (!arena_lock(a)) {
    return NULL;
  }

  uint32_t needed = ALIGN_UP(ALIGN + (uint32_t)size + FOOTER_SIZE);
  if (needed < MIN_BLOCK_SIZE) {
    needed = MIN_BLOCK_SIZE;
  }

  uint8_t start_class = get_size_class(a, (uint32_t)size);

  for (uint8_t class_idx = start_class; class_idx < NUM_SIZE_CLASSES;
       class_idx++) {
    uint32_t best_off = NULL_OFFSET;
    uint32_t best_prev_off = NULL_OFFSET;
    uint32_t best_size = 0xFFFFFFFF;

    uint32_t cur_off = a->free_lists[class_idx];
    uint32_t prev_off = NULL_OFFSET;

    while (cur_off != NULL_OFFSET) {
      Header* cur = offset_to_header(a, cur_off);
      if (!cur || !validate_block(a, cur)) {
        if (cur) {
          quarantine_block(a, cur);
        }
        rebuild_free_lists(a);
        arena_unlock(a);
        return mm_malloc_from(arena, size);
      }

      if (is_quarantined(a, cur_off, cur->size)) {
        if (prev_off == NULL_OFFSET) {
          a->free_lists[class_idx] = cur->next_free_off;
        } else {
          Header* prev = offset_to_header(a, prev_off);
          if (prev) {
            prev->next_free_off = cur->next_free_off;
            finalize_block(a, prev);
          }
        }
        cur_off = cur->next_free_off;
        continue;
      }

      if (cur->size >= needed && cur->size < best_size) {
        best_off = cur_off;
        best_prev_off = prev_off;
        best_size = cur->size;
        if (cur->size == needed) {
          break;
        }
      }

      prev_off = cur_off;
      cur_off = cur->next_free_off;
    }

    if (best_off != NULL_OFFSET) {
      Header* best = offset_to_header(a, best_off);
      if (!best) {
        continue;
      }

      uint32_t rem = best->size - needed;

      if (best_prev_off == NULL_OFFSET) {
        a->free_lists[class_idx] = best->next_free_off;
      } else {
        Header* prev = offset_to_header(a, best_prev_off);
        if (prev) {
          prev->next_free_off = best->next_free_off;
          finalize_block(a, prev);
        }
      }

      if (rem >= MIN_BLOCK_SIZE) {
        Header* r = (Header*)((uint8_t*)best + needed);
        r->magic = HEADER_MAGIC;
        r->size = rem;
        r->payload_size = 0;
        r->payload_crc = 0;
        r->allocated = 0;
        r->size_class = get_size_class(a, get_payloadCapacity(r));
        r->next_free_off = a->free_lists[r->size_class];
        finalize_block(a, r);

        if (validate_block(a, r)) {
          a->free_lists[r->size_class] = header_to_offset(a, r);
          best->size = needed;
        } else {
          quarantine_block(a, r);
        }
      }

      best->allocated = 1;
      best->payload_size = (uint32_t)size;
      best->size_class = class_idx;
      best->next_free_off = NULL_OFFSET;

      uint8_t* payload = get_payload(a, best);
      fill_pattern(a, payload, (uint32_t)size);
      best->payload_crc = compute_payload_crc(a, best);
      finalize_block(a, best);

      if (!validate_block(a, best)) {
        quarantine_block(a, best);
        continue;
      }
      arena_unlock(a);
      return payload;
    }
  }

  arena_unlock(a);
  return NULL;
}

void* mm_calloc(size_t nmemb, size_t size) {
  if (nmemb == 0 || size == 0) {
    return NULL;
  }

  // Check for overflow.
  size_t total = nmemb * size;
  if (total / nmemb != size) {
    return NULL;
  }

  void* ptr = mm_malloc(total);
  if (ptr) {
    memset(ptr, 0, total);
    // Update checksum after zeroing.
    int arena_id = find_arena_for_ptr(ptr);
    if (arena_id >= 0) {
      Arena* a = &g_arenas[arena_id];
      if (arena_lock(a)) {
        Header* h = find_header(a, ptr);
        if (h) {
          h->payload_crc = compute_payload_crc(a, h);
          finalize_block(a, h);
        }
        arena_unlock(a);
      }
    }
  }
  return ptr;
}

// Frees a previously allocated block.
// Automatically determines which arena the pointer belongs to.
// NULL pointers are safely ignored.
// Thread-safe: acquires arena lock before modifying state.
void mm_free(void* ptr) {
  // NULL is safely ignored per standard malloc behavior.
  if (!ptr) {
    return;
  }

  // Find which arena this pointer belongs to.
  // Supports cross-arena frees (allocated in one arena, freed from another).
  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return;  // Invalid pointer - not in any arena
  }

  // Cross-arena free detection: warn if freeing from different arena.
  // This is allowed but may indicate a bug in multi-threaded code.
  if ((ArenaId)arena_id != tls_current_arena) {
    // Log for debugging but allow the operation.
    #ifdef MM_DEBUG
    fprintf(stderr, "mm_free: cross-arena free detected "
            "(thread arena=%d, ptr arena=%d)\n",
            tls_current_arena, arena_id);
    #endif
  }

  // Get arena and acquire lock.
  Arena* a = &g_arenas[arena_id];
  if (!arena_lock(a)) {
    return;  // Could not acquire lock
  }

  // Verify arena is initialized.
  if (!arena_ready(a)) {
    arena_unlock(a);
    return;
  }

  // Find and validate the block header.
  Header* h = find_header(a, ptr);
  if (!h || !validate_block(a, h) || !h->allocated) {
    arena_unlock(a);
    return;  // Invalid or already-freed block
  }

  // Fill payload with pattern for brownout detection.
  // This allows detection of partial writes on next allocation.
  uint8_t* payload = get_payload(a, h);
  fill_pattern(a, payload, get_payloadCapacity(h));

  // Mark block as free and clear payload metadata.
  h->allocated = 0;
  h->payload_size = 0;
  h->payload_crc = 0;
  finalize_block(a, h);

  // Coalesce with adjacent free blocks and add to free list.
  // Coalescing reduces fragmentation.
  h = coalesce_block(a, h);
  add_to_free_list(a, h);

  arena_unlock(a);
}

int mm_is_cross_arena(void* ptr) {
  if (!ptr) {
    return 0;
  }
  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return 0;  // Invalid pointer, not cross-arena
  }
  return (ArenaId)arena_id != tls_current_arena;
}

MmStatus mm_free_checked(void* ptr) {
  if (!ptr) {
    return MM_OK;
  }

  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return MM_ERR_INVALID_PTR;
  }

  int cross_arena = ((ArenaId)arena_id != tls_current_arena);

  Arena* a = &g_arenas[arena_id];
  if (!arena_lock(a)) {
    return MM_ERR_LOCK_FAILED;
  }

  if (!arena_ready(a)) {
    arena_unlock(a);
    return MM_ERR_NOT_INITIALIZED;
  }

  Header* h = find_header(a, ptr);
  if (!h || !validate_block(a, h)) {
    arena_unlock(a);
    return MM_ERR_INVALID_PTR;
  }
  if (!h->allocated) {
    arena_unlock(a);
    return MM_ERR_NOT_ALLOCATED;  // Double-free
  }

  // Fill with pattern.
  uint8_t* payload = get_payload(a, h);
  fill_pattern(a, payload, get_payloadCapacity(h));

  // Mark as free.
  h->allocated = 0;
  h->payload_size = 0;
  h->payload_crc = 0;
  finalize_block(a, h);

  // Coalesce and add to free list.
  h = coalesce_block(a, h);
  add_to_free_list(a, h);

  arena_unlock(a);

  // Return cross-arena status (operation succeeded but was cross-arena).
  return cross_arena ? MM_ERR_INVALID_PTR : MM_OK;
}

// Safely reads data from an allocated block.
// Validates block integrity and CRC before reading.
// Detects brownout conditions (partial writes) and auto-deallocates
// corrupted blocks. Returns bytes read, or -1 on error.
int mm_read(void* ptr, size_t offset, void* buf, size_t len) {
  // Validate input parameters.
  if (!ptr || !buf) {
    return -1;
  }

  // Find which arena owns this pointer.
  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return -1;  // Pointer not in any arena
  }

  // Acquire arena lock.
  Arena* a = &g_arenas[arena_id];
  if (!arena_lock(a)) {
    return -1;
  }

  // Verify arena is ready.
  if (!arena_ready(a)) {
    arena_unlock(a);
    return -1;
  }

  // Find and validate the block, including payload CRC.
  Header* h = find_header(a, ptr);
  if (!h || !validate_with_payload(a, h) || !h->allocated) {
    arena_unlock(a);
    return -1;  // Block not found, invalid, or not allocated
  }

  // Check offset is within bounds.
  if (offset > h->payload_size) {
    arena_unlock(a);
    return -1;
  }

  // Zero-length read succeeds trivially.
  if (len == 0) {
    arena_unlock(a);
    return 0;
  }

  uint8_t* payload = get_payload(a, h);

  // Check for uninitialized pattern (unwritten or brownout).
  // This detects partial writes caused by power loss during write.
  if (g_pattern_captured && h->payload_size >= 5) {
    // Check if entire payload matches init pattern (never written).
    if (matches_pattern(a, payload, h->payload_size)) {
      arena_unlock(a);
      return -1;  // Block was allocated but never written
    }

    // Check for partial pattern (brownout indicator).
    if (has_partial_pattern(a, h)) {
      // Brownout detected: payload is partially written garbage.
      // Header+footer are valid (we passed validate_block), so we can
      // safely deallocate rather than quarantine - return memory to pool.
      h->allocated = 0;
      h->payload_size = 0;
      h->payload_crc = 0;
      finalize_block(a, h);
      h = coalesce_block(a, h);
      add_to_free_list(a, h);
      arena_unlock(a);
      return -1;  // Block was corrupted by brownout
    }
  }

  // Calculate how many bytes we can actually read.
  uint32_t avail = h->payload_size - (uint32_t)offset;
  uint32_t n = ((uint32_t)len < avail) ? (uint32_t)len : avail;

  // Copy data to user buffer.
  memcpy(buf, payload + offset, n);

  arena_unlock(a);
  return (int)n;
}

int mm_write(void* ptr, size_t offset, const void* src, size_t len) {
  if (!ptr || !src) {
    return -1;
  }

  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return -1;
  }

  Arena* a = &g_arenas[arena_id];
  if (!arena_lock(a)) {
    return -1;
  }

  if (!arena_ready(a)) {
    arena_unlock(a);
    return -1;
  }

  Header* h = find_header(a, ptr);
  if (!h || !validate_with_payload(a, h) || !h->allocated) {
    arena_unlock(a);
    return -1;
  }
  if (h->payload_size == 0) {
    arena_unlock(a);
    return -1;
  }
  if (offset != 0 || len != h->payload_size) {
    arena_unlock(a);
    return -1;
  }

  uint8_t* payload = get_payload(a, h);
  if (!range_in_heap(a, payload, h->payload_size)) {
    quarantine_block(a, h);
    arena_unlock(a);
    return -1;
  }

  memcpy(payload, src, h->payload_size);
  h->payload_crc = compute_payload_crc(a, h);
  finalize_block(a, h);

  if (!validate_with_payload(a, h)) {
    quarantine_block(a, h);
    arena_unlock(a);
    return -1;
  }

  arena_unlock(a);
  return (int)h->payload_size;
}

void* mm_realloc(void* ptr, size_t new_size) {
  if (!ptr) {
    return mm_malloc(new_size);
  }
  if (new_size == 0) {
    mm_free(ptr);
    return NULL;
  }

  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return NULL;
  }

  Arena* a = &g_arenas[arena_id];
  if (!arena_lock(a)) {
    return NULL;
  }

  if (!arena_ready(a)) {
    arena_unlock(a);
    return NULL;
  }

  Header* h = find_header(a, ptr);
  if (!h || !validate_block(a, h) || !h->allocated) {
    arena_unlock(a);
    return NULL;
  }

  uint32_t cap = get_payloadCapacity(h);
  if (new_size <= cap) {
    h->payload_size = (uint32_t)new_size;
    h->payload_crc = compute_payload_crc(a, h);
    finalize_block(a, h);
    arena_unlock(a);
    return ptr;
  }

  uint8_t* old_p = get_payload(a, h);
  uint32_t old_size = h->payload_size;
  arena_unlock(a);

  void* np = mm_malloc_from(arena_id, new_size);
  if (!np) {
    return NULL;
  }

  uint32_t copy_size = old_size < new_size ? old_size : (uint32_t)new_size;
  memcpy(np, old_p, copy_size);

  // Update checksum on new block.
  if (arena_lock(a)) {
    Header* nh = find_header(a, np);
    if (nh) {
      nh->payload_crc = compute_payload_crc(a, nh);
      finalize_block(a, nh);
    }
    arena_unlock(a);
  }

  mm_free(ptr);
  return np;
}

MmStatus mm_validate(void* ptr) {
  if (!ptr) {
    return MM_ERR_INVALID_PTR;
  }

  int arena_id = find_arena_for_ptr(ptr);
  if (arena_id < 0) {
    return MM_ERR_INVALID_PTR;
  }

  Arena* a = &g_arenas[arena_id];
  if (!arena_lock(a)) {
    return MM_ERR_LOCK_FAILED;
  }

  if (!arena_ready(a)) {
    arena_unlock(a);
    return MM_ERR_NOT_INITIALIZED;
  }

  Header* h = find_header(a, ptr);
  if (!h) {
    arena_unlock(a);
    return MM_ERR_INVALID_PTR;
  }
  if (!h->allocated) {
    arena_unlock(a);
    return MM_ERR_NOT_ALLOCATED;
  }

  Footer* f = get_footer(a, h);
  if (!f) {
    arena_unlock(a);
    return MM_ERR_CORRUPT_BLOCK;
  }

  if (h->checksum != compute_header_crc(h)) {
    arena_unlock(a);
    return MM_ERR_CHECKSUM_FAIL;
  }
  if (f->checksum != compute_footer_crc(f)) {
    arena_unlock(a);
    return MM_ERR_CHECKSUM_FAIL;
  }
  if (h->size != f->size || h->allocated != f->allocated) {
    arena_unlock(a);
    return MM_ERR_HDR_FTR_MISMATCH;
  }

  if (h->payload_size > 0) {
    if (h->payload_crc != compute_payload_crc(a, h)) {
      arena_unlock(a);
      return MM_ERR_CHECKSUM_FAIL;
    }
  }

  arena_unlock(a);
  return MM_OK;
}

MmStatus mm_arena_check(ArenaId arena, int verbose) {
  Arena* a = get_arena_by_id(arena);
  if (!a) {
    return MM_ERR_INVALID_PTR;
  }

  if (!arena_lock(a)) {
    return MM_ERR_LOCK_FAILED;
  }

  if (!arena_ready(a)) {
    if (verbose) {
      printf("ERROR: Arena %d not initialized\n", arena);
    }
    arena_unlock(a);
    return MM_ERR_NOT_INITIALIZED;
  }

  MmStatus result = MM_OK;
  uint32_t blocks_checked = 0;
  uint32_t corrupt_count = 0;
  uint32_t allocated_count = 0;
  uint32_t free_count = 0;

  if (verbose) {
    printf("=== Arena %d Check ===\n", arena);
    printf("Heap: %p, Size: %u bytes\n",
           (void*)a->heap_start, a->heap_size);
  }

  // Check for quarantined regions.
  if (a->quarantine_count > 0) {
    result = MM_ERR_QUARANTINED;
    if (verbose) {
      printf("WARNING: %u quarantined regions\n", a->quarantine_count);
    }
  }

  // Scan all blocks.
  for (uint32_t off = 0; off + MIN_BLOCK_SIZE <= a->heap_size;) {
    if (is_quarantined(a, off, HEADER_SIZE)) {
      off += ALIGN;
      continue;
    }

    Header* h = (Header*)(a->heap_start + off);
    blocks_checked++;

    if (!validate_block(a, h)) {
      corrupt_count++;
      result = MM_ERR_CORRUPT_BLOCK;
      if (verbose) {
        printf("CORRUPT block at offset %u\n", off);
      }
      off += ALIGN;
      continue;
    }

    Footer* f = get_footer(a, h);
    if (f && (h->size != f->size || h->allocated != f->allocated)) {
      result = MM_ERR_HDR_FTR_MISMATCH;
      if (verbose) {
        printf("MISMATCH at offset %u: hdr.size=%u ftr.size=%u\n",
               off, h->size, f->size);
      }
    }

    if (h->allocated) {
      allocated_count++;
    } else {
      free_count++;
    }

    off += h->size;
  }

  // Verify free list consistency.
  for (int i = 0; i < NUM_SIZE_CLASSES; i++) {
    uint32_t count = 0;
    uint32_t cur_off = a->free_lists[i];
    while (cur_off != NULL_OFFSET && count < 1000) {
      Header* cur = offset_to_header(a, cur_off);
      if (!cur || !validate_block(a, cur) || cur->allocated) {
        result = MM_ERR_FREELIST_BAD;
        if (verbose) {
          printf("FREELIST[%d] inconsistent at offset %u\n", i, cur_off);
        }
        break;
      }
      cur_off = cur->next_free_off;
      count++;
    }
  }

  if (verbose) {
    printf("Blocks checked: %u (allocated: %u, free: %u, corrupt: %u)\n",
           blocks_checked, allocated_count, free_count, corrupt_count);
    printf("Result: %s\n", mm_status_str(result));
  }

  arena_unlock(a);
  return result;
}

MmStatus mm_heap_check(int verbose) {
  MmStatus worst = MM_OK;

  for (uint8_t i = 0; i < g_num_arenas; i++) {
    MmStatus s = mm_arena_check(i, verbose);
    if (s > worst) {
      worst = s;
    }
  }

  return worst;
}

int mm_arena_defrag(ArenaId arena) {
  Arena* a = get_arena_by_id(arena);
  if (!a) {
    return 0;
  }

  if (!arena_lock(a)) {
    return 0;
  }

  if (!arena_ready(a)) {
    arena_unlock(a);
    return 0;
  }

  int coalesce_count = 0;

  // Clear free lists first.
  for (int i = 0; i < NUM_SIZE_CLASSES; i++) {
    a->free_lists[i] = NULL_OFFSET;
  }

  // Scan and coalesce adjacent free blocks.
  for (uint32_t off = 0; off + MIN_BLOCK_SIZE <= a->heap_size;) {
    if (is_quarantined(a, off, HEADER_SIZE)) {
      off += ALIGN;
      continue;
    }

    Header* h = (Header*)(a->heap_start + off);
    if (!validate_block(a, h)) {
      quarantine_add(a, off, ALIGN);
      off += ALIGN;
      continue;
    }

    if (!h->allocated) {
      // Try to coalesce with following free blocks.
      uint32_t next_off = off + h->size;
      while (next_off + MIN_BLOCK_SIZE <= a->heap_size) {
        if (is_quarantined(a, next_off, HEADER_SIZE)) {
          break;
        }
        Header* next = (Header*)(a->heap_start + next_off);
        if (!validate_block(a, next) || next->allocated) {
          break;
        }
        h->size += next->size;
        finalize_block(a, h);
        next_off = off + h->size;
        coalesce_count++;
      }

      // Add to appropriate free list.
      add_to_free_list(a, h);
    }

    off += h->size;
  }

  arena_unlock(a);
  return coalesce_count;
}

int mm_defrag(void) {
  int total = 0;
  for (uint8_t i = 0; i < g_num_arenas; i++) {
    total += mm_arena_defrag(i);
  }
  return total;
}

void mm_arena_stats(ArenaId arena) {
  Arena* a = get_arena_by_id(arena);
  if (!a) {
    printf("Invalid arena %d\n", arena);
    return;
  }

  if (!arena_lock(a)) {
    printf("Failed to lock arena %d\n", arena);
    return;
  }

  if (!arena_ready(a)) {
    printf("Arena %d not initialized.\n", arena);
    arena_unlock(a);
    return;
  }

  printf("===== ARENA %d STATS =====\n", arena);
  printf("Heap: %u bytes\n", a->heap_size);
  printf("Overhead: %zu hdr + %zu ftr = %zu bytes/block\n",
         HEADER_SIZE, FOOTER_SIZE, HEADER_SIZE + FOOTER_SIZE);
  printf("Quarantine: %u regions\n", a->quarantine_count);

  // Count blocks.
  uint32_t used = 0, freed = 0, corrupt = 0;
  uint32_t used_bytes = 0, free_bytes = 0;

  for (uint32_t off = 0; off + MIN_BLOCK_SIZE <= a->heap_size;) {
    if (is_quarantined(a, off, HEADER_SIZE)) {
      off += ALIGN;
      continue;
    }
    Header* h = (Header*)(a->heap_start + off);
    if (!validate_block(a, h)) {
      corrupt++;
      off += ALIGN;
      continue;
    }
    if (h->allocated) {
      used++;
      used_bytes += h->payload_size;
    } else {
      freed++;
      free_bytes += get_payloadCapacity(h);
    }
    off += h->size;
  }

  printf("Blocks: %u allocated, %u free, %u corrupt\n", used, freed, corrupt);
  printf("Payload: %u bytes used, %u bytes free\n", used_bytes, free_bytes);

  // Free list stats.
  printf("Size classes: [");
  for (int i = 0; i < NUM_SIZE_CLASSES - 1; i++) {
    printf("%u%s", a->size_classes[i], i < NUM_SIZE_CLASSES - 2 ? ", " : "");
  }
  printf(", ∞]\n");  // Whatever is leftover

  printf("Free lists: ");
  for (int i = 0; i < NUM_SIZE_CLASSES; i++) {
    uint32_t count = 0;
    uint32_t cur = a->free_lists[i];
    while (cur != NULL_OFFSET && count < 100) {
      Header* h = offset_to_header(a, cur);
      if (!h) {
        break;
      }
      cur = h->next_free_off;
      count++;
    }
    if (count > 0) {
      printf("[%u]=%u ", a->size_classes[i], count);
    }
  }
  printf("\n");
  printf("==============================\n");

  arena_unlock(a);
}

void mm_heap_stats(void) {
  printf("===== HEAP STATS (%u arenas) =====\n", g_num_arenas);
  for (uint8_t i = 0; i < g_num_arenas; i++) {
    mm_arena_stats(i);
  }
}

int mm_ptr_arena(void* ptr) {
  return find_arena_for_ptr(ptr);
}
