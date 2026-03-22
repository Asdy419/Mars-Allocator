# Mars-Allocator

A fault-tolerant memory allocator for embedded systems operating in high-radiation environments, such as Mars rovers. Provides a hardened `malloc`/`free` API with corruption detection, radiation resilience, and thread-safe multi-arena allocation.

---

## Features

- **Radiation Resilience** — CRC16-CCITT checksums on every block; bit-flip simulation via `simulate_storm()`
- **Corruption Recovery** — Header/footer mirroring allows recovery after partial corruption; quarantine system isolates damaged regions
- **Thread Safety** — Per-arena mutex locking with up to 8 independent arenas
- **Segregated Free Lists** — 9 dynamically-computed size classes for O(1) common-case allocation
- **Block Coalescing** — Adjacent free block merging to reduce fragmentation
- **Brownout Tolerance** — Survives simulated power-loss events
- **Safe I/O** — Bounds-checked `mm_read`/`mm_write` for accessing allocated blocks

---

## Project Structure

```
Mars-Allocator/
├── allocator.h        # Public API
├── allocator.c        # Core allocator implementation
├── runme.c            # Test suite and driver
└── Makefile           # Build configuration
```

---

## Building

Requires GCC and a C11-compatible toolchain.

```bash
make all      # Builds liballocator.so and the runme test executable
make clean    # Removes build artifacts
```

---

## Running Tests

```bash
./runme                            # Default: 8192-byte heap, 1 arena
./runme --size <N>                 # Custom heap size in bytes
./runme --arenas <N>               # Number of arenas (1–8)
./runme --storm <N>                # Radiation storm intensity (1–4)
./runme --seed <N>                 # Random seed for reproducibility
```

**Example:**
```bash
./runme --size 16384 --arenas 4 --storm 2
```

---

## API Reference

### Core Allocation

| Function | Description |
|---|---|
| `mm_malloc(size)` | Allocate `size` bytes |
| `mm_calloc(nmemb, size)` | Allocate and zero-initialize |
| `mm_realloc(ptr, size)` | Resize an existing allocation |
| `mm_free(ptr)` | Free an allocation (double-free safe) |

### Arena Operations

| Function | Description |
|---|---|
| `mm_arena_init(heap, size, num_arenas)` | Initialize allocator with N arenas |
| `mm_arena_select(arena)` | Assign current thread to an arena |
| `mm_malloc_from(arena, size)` | Allocate from a specific arena |
| `mm_free_checked(ptr)` | Free with status return code |

### Safe I/O

| Function | Description |
|---|---|
| `mm_read(ptr, offset, buf, len)` | Bounds-checked read from allocation |
| `mm_write(ptr, offset, src, len)` | Bounds-checked write to allocation |

### Diagnostics

| Function | Description |
|---|---|
| `mm_validate(ptr)` | Validate a single block |
| `mm_heap_check(verbose)` | Full heap integrity scan |
| `mm_arena_check(arena, verbose)` | Check a specific arena |
| `mm_defrag()` | Coalesce all free blocks |
| `mm_heap_stats()` | Print heap statistics |
| `mm_ptr_arena(ptr)` | Find which arena owns a pointer |

---

## Status Codes

```c
MM_OK                    // Success
MM_ERR_CORRUPT_BLOCK     // Corruption detected
MM_ERR_QUARANTINED       // Quarantined regions exist
MM_ERR_FREELIST_BAD      // Free list inconsistency
MM_ERR_HDR_FTR_MISMATCH  // Header/footer mismatch
MM_ERR_INVALID_PTR       // Invalid pointer
MM_ERR_NOT_ALLOCATED     // Block not in allocated state
MM_ERR_CHECKSUM_FAIL     // CRC verification failed
MM_ERR_OUT_OF_MEMORY     // Allocation failed
MM_ERR_INVALID_SIZE      // Invalid size parameter
MM_ERR_NOT_INITIALIZED   // Heap not initialized
MM_ERR_LOCK_FAILED       // Mutex acquisition failed
```

---

## Block Layout

Each allocation has a 32-byte overhead split between a header and footer:

```
[ Header (20 bytes) | Padding (20 bytes) | Payload (N bytes) | Footer (12 bytes) ]
```

- **Header**: magic, CRC16, size, payload size, checksum, allocation status, size class, next free offset
- **Footer**: magic, CRC16, size, payload CRC, allocation status
- All payloads are **40-byte aligned**

---

## Configuration

Key compile-time constants in `allocator.c`:

| Constant | Default | Description |
|---|---|---|
| `ALIGN` | `40` | Payload alignment |
| `NUM_SIZE_CLASSES` | `9` | Segregated free list count |
| `MIN_CLASS_PAYLOAD` | `40` | Smallest size class (bytes) |
| `MAX_QUARANTINE` | `16` | Max quarantine entries |
| `MM_ARENA_MAX` | `8` | Maximum number of arenas |
| `HEADER_MAGIC` | `0xC0FE` | Header integrity marker |
| `FOOTER_MAGIC` | `0xBEEF` | Footer integrity marker |

---

## Test Coverage

The test suite in `runme.c` covers:

- Basic allocation and freeing
- Zero-initialization (`calloc`) and overflow protection
- Block validation and heap integrity checking
- Defragmentation and block coalescing
- Safe read/write bounds checking
- Double-free detection
- Best-fit size class selection
- Multi-threaded concurrent allocation
- Cross-arena free detection
- Radiation storm resilience (4 intensity levels)
- Brownout / power-loss recovery
- Corruption recovery mechanisms
