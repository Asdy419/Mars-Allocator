// allocator.h - Hardened Memory Allocator for Mars Rover
//
// A fault-tolerant malloc/free system with:
// - Segregated free lists for O(1) common-case allocation
// - Thread-safe heap arenas with per-arena locking
// - Comprehensive integrity checking
// - Radiation and brownout resilience

#ifndef ALLOCATOR_H_
#define ALLOCATOR_H_

#include <stdint.h>
#include <stddef.h>

// Status codes returned by validation and diagnostic functions.
typedef enum {
  MM_OK = 0,                    // No errors detected
  MM_ERR_CORRUPT_BLOCK = 1,     // Corrupt block(s) found
  MM_ERR_QUARANTINED = 2,       // Quarantined regions exist
  MM_ERR_FREELIST_BAD = 3,      // Free list inconsistency
  MM_ERR_HDR_FTR_MISMATCH = 4,  // Header/footer mismatch
  MM_ERR_INVALID_PTR = 5,       // Invalid pointer provided
  MM_ERR_NOT_ALLOCATED = 6,     // Block not allocated
  MM_ERR_CHECKSUM_FAIL = 7,     // Checksum verification failed
  MM_ERR_OUT_OF_MEMORY = 8,     // Allocation failed
  MM_ERR_INVALID_SIZE = 9,      // Invalid size parameter
  MM_ERR_NOT_INITIALIZED = 10,  // Heap not initialized
  MM_ERR_LOCK_FAILED = 11       // Failed to acquire lock
} MmStatus;

// Arena ID for thread-safe allocation (0 = default/global arena).
typedef uint8_t ArenaId;

#define MM_ARENA_DEFAULT 0
#define MM_ARENA_MAX 8

// Initializes the allocator over a provided memory block (single arena).
// Returns MM_OK on success, error code on failure.
MmStatus mm_init(uint8_t* heap, size_t heap_size);

// Initializes multiple arenas within the heap for thread isolation.
// Divides heap into num_arenas equal parts.
// Each arena has its own lock for thread safety.
MmStatus mm_arena_init(uint8_t* heap, size_t heap_size, uint8_t num_arenas);

// Selects the arena for subsequent allocations on this thread.
// Thread-local: each thread can select its own arena independently.
void mm_arena_select(ArenaId arena);

// Gets the currently selected arena for this thread.
ArenaId mm_arena_current(void);

// Allocates memory with 40-byte aligned payload.
// Returns pointer to payload, or NULL on failure.
void* mm_malloc(size_t size);

// Allocates from a specific arena (explicit arena selection).
// Useful for cross-thread allocations.
void* mm_malloc_from(ArenaId arena, size_t size);

// Allocates and zero-initializes memory for an array.
// Returns pointer to payload, or NULL on failure.
void* mm_calloc(size_t nmemb, size_t size);

// Frees a previously-allocated pointer.
// Automatically determines which arena the pointer belongs to.
// NULL is safely ignored. Detects double-free attempts.
void mm_free(void* ptr);

// Frees with status return. Returns MM_OK on success.
// Returns MM_ERR_INVALID_PTR if cross-arena free detected (still frees).
MmStatus mm_free_checked(void* ptr);

// Checks if freeing ptr would be a cross-arena operation.
// Returns 1 if ptr belongs to a different arena than current thread's.
int mm_is_cross_arena(void* ptr);

// Safely reads data from an allocated block.
// Returns bytes read, or -1 on error.
int mm_read(void* ptr, size_t offset, void* buf, size_t len);

// Safely writes data into an allocated block.
// Returns bytes written, or -1 on error.
int mm_write(void* ptr, size_t offset, const void* src, size_t len);

// Resizes a previously allocated block.
// Returns pointer to resized block, or NULL on failure.
void* mm_realloc(void* ptr, size_t new_size);

// Validates a single pointer/block.
// Returns MM_OK if valid, specific error code otherwise.
MmStatus mm_validate(void* ptr);

// Checks entire heap integrity (all arenas).
// If verbose != 0, prints detailed diagnostics.
// Returns MM_OK if heap is healthy, error code otherwise.
MmStatus mm_heap_check(int verbose);

// Checks a specific arena's integrity.
MmStatus mm_arena_check(ArenaId arena, int verbose);

// Defragments all arenas by coalescing adjacent free blocks.
// Returns total number of coalesce operations performed.
int mm_defrag(void);

// Defragments a specific arena.
int mm_arena_defrag(ArenaId arena);

// Outputs current heap usage and integrity statistics.
void mm_heap_stats(void);

// Outputs statistics for a specific arena.
void mm_arena_stats(ArenaId arena);

// Returns string description of status code.
const char* mm_status_str(MmStatus status);

// Returns which arena a pointer belongs to, or -1 if invalid.
int mm_ptr_arena(void* ptr);

#endif  // ALLOCATOR_H_
