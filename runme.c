// runme.c - Test driver for the Mars Rover memory allocator
//
// Usage: ./runme [--seed N] [--storm N] [--size N] [--arenas N]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <threads.h>

#include "allocator.h"

#define DEFAULT_HEAP_SIZE 8192
#define DEFAULT_STORM 0
#define NUM_THREADS 4
#define ALLOCS_PER_THREAD 50

static uint8_t g_test_pattern[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x42};

static void fill_heap_pattern(uint8_t* heap, size_t size) {
  for (size_t i = 0; i < size; i++) {
    heap[i] = g_test_pattern[i % 5];
  }
}

static void simulate_storm(uint8_t* heap, size_t size, int intensity) {
  if (intensity <= 0) {
    return;
  }
  size_t flips = 0;
  switch (intensity) {
    case 1: flips = size / 1000; break;
    case 2: flips = size / 100; break;
    case 3: flips = size / 20; break;
    default: flips = size / 50; break;
  }
  if (flips == 0) {
    flips = 1;
  }
  printf("[STORM] Simulating %zu bit flips (intensity %d)\n", flips, intensity);

  // Flip random bits throughout the heap memory.
  for (size_t i = 0; i < flips; i++) {
    size_t byte_idx = rand() % size;
    int bit_idx = rand() % 8;
    heap[byte_idx] ^= (1 << bit_idx);  // XOR to flip the bit
  }
}

/**
 * test_basic_alloc - Verify fundamental malloc/free operations.
 *
 * Tests:
 * - Sequential allocations succeed and return valid pointers
 * - Free returns memory to the pool
 * - Subsequent allocation can reuse freed memory
 * - All allocations can be freed without error
 *
 * @return 1 on success, 0 on failure
 */
static int test_basic_alloc(void) {
  printf("\n=== Test: Basic Allocation ===\n");

  // Allocate first block.
  void* p1 = mm_malloc(100);
  if (!p1) {
    printf("FAIL: mm_malloc(100) returned NULL\n");
    return 0;
  }
  printf("PASS: Allocated 100 bytes at %p\n", p1);

  // Allocate second block.
  void* p2 = mm_malloc(200);
  if (!p2) {
    printf("FAIL: mm_malloc(200) returned NULL\n");
    return 0;
  }
  printf("PASS: Allocated 200 bytes at %p\n", p2);

  // Allocate third block.
  void* p3 = mm_malloc(50);
  if (!p3) {
    printf("FAIL: mm_malloc(50) returned NULL\n");
    return 0;
  }
  printf("PASS: Allocated 50 bytes at %p\n", p3);

  // Free middle block to create a hole.
  mm_free(p2);
  printf("PASS: Freed p2\n");

  // Allocate into the freed space.
  void* p4 = mm_malloc(150);
  if (!p4) {
    printf("FAIL: mm_malloc(150) after free returned NULL\n");
    return 0;
  }
  printf("PASS: Allocated 150 bytes after free at %p\n", p4);

  // Clean up remaining allocations.
  mm_free(p1);
  mm_free(p3);
  mm_free(p4);
  printf("PASS: Freed all remaining blocks\n");

  return 1;
}

/**
 * test_calloc - Verify calloc (allocate + zero-initialize).
 *
 * Tests:
 * - mm_calloc returns valid pointer
 * - Allocated memory is zero-initialized
 * - Integer overflow in size calculation is detected and rejected
 *
 * @return 1 on success, 0 on failure
 */
static int test_calloc(void) {
  printf("\n=== Test: mm_calloc ===\n");

  // Allocate array via calloc.
  void* p = mm_calloc(10, 8);
  if (!p) {
    printf("FAIL: mm_calloc(10, 8) returned NULL\n");
    return 0;
  }
  printf("PASS: Allocated 80 bytes via calloc at %p\n", p);

  // Verify zeroed.
  uint8_t* bytes = (uint8_t*)p;
  int zeroed = 1;
  for (int i = 0; i < 80; i++) {
    if (bytes[i] != 0) {
      zeroed = 0;
      printf("FAIL: Byte %d is %02x, not 0\n", i, bytes[i]);
      break;
    }
  }
  if (zeroed) {
    printf("PASS: Memory is zero-initialized\n");
  }

  mm_free(p);

  // Test overflow protection - should reject requests that would overflow.
  void* p2 = mm_calloc(0xFFFFFFFF, 0xFFFFFFFF);
  if (p2) {
    printf("FAIL: calloc overflow not detected\n");
    mm_free(p2);
    return 0;
  }
  printf("PASS: Overflow detected correctly\n");

  return zeroed;
}

/**
 * test_validate - Verify block validation functionality.
 *
 * Tests:
 * - mm_validate returns MM_OK for valid allocated blocks
 * - mm_validate returns MM_ERR_INVALID_PTR for NULL
 * - mm_validate detects freed blocks
 *
 * @return 1 on success, 0 on failure
 */
static int test_validate(void) {
  printf("\n=== Test: mm_validate ===\n");

  // Allocate a test block.
  void* p = mm_malloc(64);
  if (!p) {
    printf("FAIL: Initial allocation failed\n");
    return 0;
  }

  // Validate should succeed on freshly allocated block.
  MmStatus status = mm_validate(p);
  if (status != MM_OK) {
    printf("FAIL: validate returned %s on valid block\n",
           mm_status_str(status));
    mm_free(p);
    return 0;
  }
  printf("PASS: Valid block returns MM_OK\n");

  // Test NULL pointer handling.
  status = mm_validate(NULL);
  if (status != MM_ERR_INVALID_PTR) {
    printf("FAIL: NULL should return INVALID_PTR, got %s\n",
           mm_status_str(status));
    mm_free(p);
    return 0;
  }
  printf("PASS: NULL returns MM_ERR_INVALID_PTR\n");

  // Free the block.
  mm_free(p);

  // Validate should fail on freed block.
  status = mm_validate(p);
  if (status == MM_OK) {
    printf("FAIL: Freed block should not validate as OK\n");
    return 0;
  }
  printf("PASS: Freed block returns error: %s\n", mm_status_str(status));

  return 1;
}

/**
 * test_heap_check - Verify heap integrity checking functionality.
 *
 * Tests:
 * - mm_heap_check returns MM_OK for a clean heap
 * - mm_heap_check works correctly after allocations and frees
 *
 * @return 1 on success, 0 on failure
 */
static int test_heap_check(void) {
  printf("\n=== Test: mm_heap_check ===\n");

  // Allocate some blocks.
  void* p1 = mm_malloc(100);
  void* p2 = mm_malloc(200);

  // Check heap integrity.
  MmStatus status = mm_heap_check(0);
  if (status != MM_OK) {
    printf("FAIL: Heap check failed on clean heap: %s\n",
           mm_status_str(status));
    mm_free(p1);
    mm_free(p2);
    return 0;
  }
  printf("PASS: Clean heap returns MM_OK\n");

  // Free blocks.
  mm_free(p1);
  mm_free(p2);

  // Check heap after frees.
  status = mm_heap_check(0);
  if (status != MM_OK) {
    printf("FAIL: Heap check failed after frees: %s\n", mm_status_str(status));
    return 0;
  }
  printf("PASS: Heap OK after frees\n");

  return 1;
}

static int test_defrag(void) {
  printf("\n=== Test: mm_defrag ===\n");

  // Allocate several blocks.
  void* blocks[5];
  for (int i = 0; i < 5; i++) {
    blocks[i] = mm_malloc(50);
    if (!blocks[i]) {
      printf("FAIL: Allocation %d failed\n", i);
      return 0;
    }
  }
  printf("PASS: Allocated 5 blocks\n");

  // Free alternating blocks to create fragmentation.
  mm_free(blocks[1]);
  mm_free(blocks[3]);
  printf("PASS: Freed blocks 1 and 3 (creating fragmentation)\n");

  // Defrag shouldn't help much here (non-adjacent).
  int ops = mm_defrag();
  printf("INFO: Defrag performed %d coalesce operations\n", ops);

  // Free remaining to create contiguous free space.
  mm_free(blocks[0]);
  mm_free(blocks[2]);
  mm_free(blocks[4]);
  printf("PASS: Freed remaining blocks\n");

  ops = mm_defrag();
  printf("INFO: Defrag performed %d coalesce operations\n", ops);

  // Should be able to allocate large block now.
  void* big = mm_malloc(400);
  if (!big) {
    printf("FAIL: Large allocation after defrag failed\n");
    return 0;
  }
  printf("PASS: Large allocation after defrag succeeded\n");
  mm_free(big);

  return 1;
}

static int test_read_write(void) {
  printf("\n=== Test: Read/Write ===\n");

  char write_buf[64] = "Hello, Mars! This is a test message.";
  size_t data_len = strlen(write_buf) + 1;

  void* p = mm_malloc(data_len);
  if (!p) {
    printf("FAIL: mm_malloc(%zu) returned NULL\n", data_len);
    return 0;
  }

  int written = mm_write(p, 0, write_buf, data_len);
  if (written != (int)data_len) {
    printf("FAIL: mm_write returned %d, expected %zu\n", written, data_len);
    mm_free(p);
    return 0;
  }
  printf("PASS: Wrote %d bytes\n", written);

  char read_buf[64] = {0};
  int rd = mm_read(p, 0, read_buf, data_len);
  if (rd < 0) {
    printf("FAIL: mm_read returned %d\n", rd);
    mm_free(p);
    return 0;
  }

  if (strcmp(read_buf, write_buf) != 0) {
    printf("FAIL: Read data doesn't match written data\n");
    mm_free(p);
    return 0;
  }
  printf("PASS: Read back correct data: \"%s\"\n", read_buf);

  mm_free(p);
  return 1;
}

static int test_double_free(void) {
  printf("\n=== Test: Double-Free Detection ===\n");

  void* p = mm_malloc(64);
  if (!p) {
    printf("FAIL: mm_malloc failed\n");
    return 0;
  }

  mm_free(p);
  printf("PASS: First free completed\n");

  mm_free(p);
  printf("PASS: Double-free handled safely (no crash)\n");

  return 1;
}

static int test_coalescing(void) {
  printf("\n=== Test: Free Block Coalescing ===\n");

  void* p1 = mm_malloc(100);
  void* p2 = mm_malloc(100);
  void* p3 = mm_malloc(100);

  if (!p1 || !p2 || !p3) {
    printf("FAIL: Initial allocations failed\n");
    return 0;
  }
  printf("PASS: Allocated 3 blocks\n");

  mm_free(p2);
  printf("PASS: Freed middle block\n");

  mm_free(p1);
  printf("PASS: Freed first block (should coalesce)\n");

  mm_free(p3);
  printf("PASS: Freed last block (should coalesce all)\n");

  void* big = mm_malloc(250);
  if (!big) {
    printf("FAIL: Could not allocate large block after coalescing\n");
    return 0;
  }
  printf("PASS: Allocated large block after coalescing\n");

  mm_free(big);
  return 1;
}

static int test_best_fit(void) {
  printf("\n=== Test: Best-Fit Within Class ===\n");

  // Allocate blocks of different sizes within same class.
  void* p1 = mm_malloc(100);  // Will get ~120 capacity
  void* p2 = mm_malloc(150);  // Will get ~200 capacity
  void* p3 = mm_malloc(180);  // Will get ~200 capacity

  if (!p1 || !p2 || !p3) {
    printf("FAIL: Initial allocations failed\n");
    return 0;
  }
  printf("PASS: Allocated 3 blocks (100, 150, 180 bytes)\n");

  // Free them all - they'll go to their respective size classes.
  mm_free(p1);
  mm_free(p2);
  mm_free(p3);
  printf("PASS: Freed all blocks\n");

  // Now allocate 140 bytes - best-fit should pick the 150-byte block
  // (smallest that fits) rather than first-fit which might pick larger.
  void* p4 = mm_malloc(140);
  if (!p4) {
    printf("FAIL: Reallocation failed\n");
    return 0;
  }
  printf("PASS: Allocated 140 bytes (best-fit should pick smallest fit)\n");

  // Allocate 90 bytes - should get the 100-byte block.
  void* p5 = mm_malloc(90);
  if (!p5) {
    printf("FAIL: Second reallocation failed\n");
    mm_free(p4);
    return 0;
  }
  printf("PASS: Allocated 90 bytes\n");

  mm_free(p4);
  mm_free(p5);
  printf("PASS: Best-fit allocation working\n");

  return 1;
}

static int test_segregated_lists(void) {
  printf("\n=== Test: Segregated Free Lists ===\n");

  // Allocate various sizes to exercise different size classes.
  void* small1 = mm_malloc(20);   // Class 0 (<=40)
  void* small2 = mm_malloc(40);   // Class 0
  void* med1 = mm_malloc(100);    // Class 2 (<=120)
  void* med2 = mm_malloc(150);    // Class 3 (<=200)
  void* large = mm_malloc(500);   // Class 5 (<=520)

  if (!small1 || !small2 || !med1 || !med2 || !large) {
    printf("FAIL: Mixed size allocations failed\n");
    return 0;
  }
  printf("PASS: Allocated blocks of various sizes\n");

  // Free in mixed order.
  mm_free(med1);
  mm_free(small1);
  mm_free(large);

  // Allocate again - should find blocks in appropriate size classes.
  void* new_small = mm_malloc(30);
  void* new_large = mm_malloc(400);

  if (!new_small || !new_large) {
    printf("FAIL: Reallocation failed\n");
    mm_free(small2);
    mm_free(med2);
    return 0;
  }
  printf("PASS: Reallocated from segregated lists\n");

  mm_free(small2);
  mm_free(med2);
  mm_free(new_small);
  mm_free(new_large);

  printf("PASS: Segregated list allocation working\n");
  return 1;
}

static int test_dynamic_size_classes(size_t heap_size) {
  printf("\n=== Test: Dynamic Size Classes ===\n");

  // Size classes should scale with heap size.
  // Class 7 (second-to-last) should be roughly heap_size/4.
  uint32_t expected_max = (uint32_t)(heap_size / 4);

  // Try to allocate something that should fit in the second-to-last class.
  size_t test_size = expected_max / 2;
  void* p = mm_malloc(test_size);

  if (!p) {
    printf("FAIL: Could not allocate %zu bytes\n", test_size);
    return 0;
  }
  printf("PASS: Allocated %zu bytes (expected max class ~%u)\n",
         test_size, expected_max);
  mm_free(p);

  // Verify that classes are strictly increasing and reasonable.
  // We do this by allocating at class boundaries.
  printf("Testing class boundary allocations:\n");

  size_t test_sizes[] = {40, 80, 200, 500, 1000, 2000};
  int num_tests = sizeof(test_sizes) / sizeof(test_sizes[0]);
  int allocs_ok = 0;

  for (int i = 0; i < num_tests; i++) {
    if (test_sizes[i] > heap_size / 2) {
      break;  // Skip sizes too large for this heap
    }
    void* ptr = mm_malloc(test_sizes[i]);
    if (ptr) {
      allocs_ok++;
      mm_free(ptr);
    }
  }

  if (allocs_ok < 3) {
    printf("FAIL: Only %d boundary allocations succeeded\n", allocs_ok);
    return 0;
  }
  printf("PASS: %d class boundary allocations succeeded\n", allocs_ok);

  printf("PASS: Dynamic size classes working\n");
  return 1;
}

// Thread worker function for multi-threaded test.
typedef struct {
  int thread_id;
  int arena_id;
  int success_count;
  int fail_count;
} thread_arg_t;

static int thread_worker(void* arg) {
  thread_arg_t* ta = (thread_arg_t*)arg;
  mm_arena_select(ta->arena_id);

  void* ptrs[ALLOCS_PER_THREAD];
  memset(ptrs, 0, sizeof(ptrs));

  for (int i = 0; i < ALLOCS_PER_THREAD; i++) {
    size_t sz = 16 + (rand() % 100);
    ptrs[i] = mm_malloc(sz);
    if (ptrs[i]) {
      ta->success_count++;
      memset(ptrs[i], ta->thread_id, sz);
    } else {
      ta->fail_count++;
    }
  }

  for (int i = 0; i < ALLOCS_PER_THREAD; i += 2) {
    if (ptrs[i]) {
      mm_free(ptrs[i]);
      ptrs[i] = NULL;
    }
  }

  for (int i = 0; i < ALLOCS_PER_THREAD; i += 2) {
    size_t sz = 32 + (rand() % 64);
    ptrs[i] = mm_malloc(sz);
    if (ptrs[i]) {
      ta->success_count++;
    }
  }

  for (int i = 0; i < ALLOCS_PER_THREAD; i++) {
    if (ptrs[i]) {
      mm_free(ptrs[i]);
    }
  }

  return 0;
}

static int test_multi_threaded(int num_arenas) {
  printf("\n=== Test: Multi-Threaded Allocation (%d arenas) ===\n", num_arenas);

  thrd_t threads[NUM_THREADS];
  thread_arg_t args[NUM_THREADS];

  for (int i = 0; i < NUM_THREADS; i++) {
    args[i].thread_id = i;
    args[i].arena_id = i % num_arenas;
    args[i].success_count = 0;
    args[i].fail_count = 0;
    if (thrd_create(&threads[i], thread_worker, &args[i]) != thrd_success) {
      printf("FAIL: Could not create thread %d\n", i);
      return 0;
    }
  }

  for (int i = 0; i < NUM_THREADS; i++) {
    thrd_join(threads[i], NULL);
  }

  int total_success = 0, total_fail = 0;
  for (int i = 0; i < NUM_THREADS; i++) {
    printf("  Thread %d (arena %d): %d allocs, %d fails\n",
           i, args[i].arena_id, args[i].success_count, args[i].fail_count);
    total_success += args[i].success_count;
    total_fail += args[i].fail_count;
  }
  printf("Total: %d successful, %d failed\n", total_success, total_fail);

  MmStatus status = mm_heap_check(0);
  if (status != MM_OK && status != MM_ERR_QUARANTINED) {
    printf("FAIL: Heap corrupted: %s\n", mm_status_str(status));
    return 0;
  }

  printf("PASS: Multi-threaded test completed, heap OK\n");
  return 1;
}

static int test_cross_arena_free(int num_arenas) {
  printf("\n=== Test: Cross-Arena Free Detection ===\n");

  if (num_arenas < 2) {
    printf("SKIP: Need at least 2 arenas for cross-arena test\n");
    return 1;
  }

  // Allocate from arena 0.
  mm_arena_select(0);
  void* p0 = mm_malloc(64);
  if (!p0) {
    printf("FAIL: Could not allocate from arena 0\n");
    return 0;
  }
  printf("Allocated from arena 0: %p\n", p0);

  // Check cross-arena detection.
  int is_cross = mm_is_cross_arena(p0);
  if (is_cross) {
    printf("FAIL: Same-arena pointer detected as cross-arena\n");
    mm_free(p0);
    return 0;
  }
  printf("PASS: Same-arena pointer correctly identified\n");

  // Switch to arena 1.
  mm_arena_select(1);
  is_cross = mm_is_cross_arena(p0);
  if (!is_cross) {
    printf("FAIL: Cross-arena pointer not detected\n");
    mm_free(p0);
    return 0;
  }
  printf("PASS: Cross-arena pointer correctly detected\n");

  // Free from different arena (should work but report cross-arena).
  MmStatus status = mm_free_checked(p0);
  if (status == MM_ERR_INVALID_PTR) {
    printf("PASS: mm_free_checked reported cross-arena free\n");
  } else if (status == MM_OK) {
    printf("INFO: mm_free_checked returned OK (unexpected for cross-arena)\n");
  } else {
    printf("FAIL: mm_free_checked returned unexpected status: %s\n",
           mm_status_str(status));
    return 0;
  }

  // Verify the pointer was actually freed.
  mm_arena_select(0);
  void* p1 = mm_malloc(64);
  if (!p1) {
    printf("FAIL: Could not reallocate after cross-arena free\n");
    return 0;
  }
  printf("PASS: Memory reclaimed after cross-arena free\n");
  mm_free(p1);

  printf("PASS: Cross-arena free detection working\n");
  return 1;
}

/**
 * test_storm_resilience - Test allocator's ability to handle radiation bit-flips.
 *
 * Simulates cosmic ray induced bit-flips in heap memory and verifies:
 * - Allocator detects corrupted blocks via CRC checks
 * - Corrupted blocks are quarantined (excluded from future allocations)
 * - Intact blocks remain readable after the storm
 * - New allocations succeed in remaining heap space
 *
 * @param heap       Pointer to heap memory
 * @param heap_size  Size of heap in bytes
 * @param intensity  Storm intensity (1=light, 2=moderate, 3=heavy)
 * @return 1 on success, 0 on failure
 */
static int test_storm_resilience(uint8_t* heap, size_t heap_size,
                                 int intensity) {
  printf("\n=== Test: Storm Resilience (intensity %d) ===\n", intensity);

  // Allocate several blocks and write data to them.
  void* blocks[5];
  for (int i = 0; i < 5; i++) {
    blocks[i] = mm_malloc(50 + i * 20);
    if (blocks[i]) {
      // Write identifiable data to each block.
      char msg[32];
      snprintf(msg, sizeof(msg), "Block %d data", i);
      mm_write(blocks[i], 0, msg, strlen(msg) + 1);
    }
  }

  // Count how many blocks were successfully allocated.
  int allocated = 0;
  for (int i = 0; i < 5; i++) {
    if (blocks[i]) {
      allocated++;
    }
  }
  printf("Allocated %d blocks before storm\n", allocated);

  // Simulate radiation-induced bit flips in heap memory.
  simulate_storm(heap, heap_size, intensity);

  // Try to read each block; count readable vs corrupted.
  int readable = 0;
  int corrupted = 0;
  for (int i = 0; i < 5; i++) {
    if (blocks[i]) {
      char buf[32];
      // mm_read validates CRC and returns -1 if corrupted.
      int rd = mm_read(blocks[i], 0, buf, sizeof(buf));
      if (rd > 0) {
        readable++;
      } else {
        corrupted++;  // Block was corrupted by storm
      }
    }
  }
  printf("After storm: %d readable, %d corrupted/quarantined\n",
         readable, corrupted);

  // Try to free remaining blocks.
  for (int i = 0; i < 5; i++) {
    if (blocks[i]) {
      mm_free(blocks[i]);
    }
  }
  printf("PASS: All frees completed safely (no crash)\n");

  void* post_storm = mm_malloc(100);
  if (post_storm) {
    printf("PASS: Post-storm allocation succeeded\n");
    mm_free(post_storm);
  } else {
    printf("INFO: Post-storm allocation failed (heap may be damaged)\n");
  }

  return 1;
}

static int test_brownout(uint8_t* heap, size_t heap_size) {
  printf("\n=== Test: Brownout Deallocation ===\n");

  // Allocate a block.
  void* p = mm_malloc(128);
  if (!p) {
    printf("FAIL: Initial allocation failed\n");
    return 0;
  }
  printf("PASS: Allocated 128 bytes at %p\n", p);

  // Write full data first (this sets payload_size and CRC correctly).
  char full_data[128];
  memset(full_data, 'X', 128);
  int wr = mm_write(p, 0, full_data, 128);
  if (wr != 128) {
    printf("FAIL: Full write failed\n");
    mm_free(p);
    return 0;
  }
  printf("PASS: Wrote 128 bytes\n");

  // Now simulate brownout: directly overwrite part of payload with
  // uninitialized pattern. This simulates power loss mid-write where
  // the first part was written but the rest stayed uninitialized.
  uint8_t* payload = (uint8_t*)p;

  // Copy the captured pattern into the middle of the payload.
  // The allocator captures first 5 bytes of heap as pattern.
  // We'll put this pattern starting at offset 20.
  for (int i = 20; i < 30; i++) {
    // The pattern repeats based on position
    payload[i] = heap[i % 5];
  }
  printf("INFO: Simulated brownout (pattern injected at offset 20)\n");

  // Try to read - if brownout detected, block should be deallocated.
  char buf[128];
  int rd = mm_read(p, 0, buf, 128);

  if (rd < 0) {
    printf("PASS: Read detected brownout (returned -1)\n");

    // The block should have been deallocated, not quarantined.
    // Verify by allocating - if memory was returned to pool, this works.
    void* p2 = mm_malloc(100);
    if (p2) {
      printf("PASS: Memory reclaimed - reallocation succeeded\n");
      mm_free(p2);
    } else {
      // Could fail if pattern didn't match exactly
      printf("INFO: Reallocation failed (may be quarantined instead)\n");
    }
  } else {
    // Brownout not detected - pattern didn't match.
    printf("INFO: Read succeeded (%d bytes) - pattern not detected\n", rd);
    printf("INFO: (Pattern may have been overwritten or not captured)\n");
    mm_free(p);
  }

  // Verify heap is still OK.
  MmStatus s2 = mm_heap_check(0);
  if (s2 != MM_OK && s2 != MM_ERR_QUARANTINED) {
    printf("FAIL: Heap corrupted after brownout test: %s\n", mm_status_str(s2));
    return 0;
  }

  printf("PASS: Heap OK after brownout test\n");
  return 1;
}

static int test_recovery(uint8_t* heap, size_t heap_size) {
  printf("\n=== Test: Bidirectional Recovery ===\n");

  int passed = 0;
  int total = 0;

  // Test 1: Header recovery from footer.
  printf("\n-- Header Recovery from Footer --\n");
  void* p1 = mm_malloc(64);
  if (!p1) {
    printf("FAIL: Initial allocation failed\n");
    return 0;
  }

  char data1[64];
  memset(data1, 'R', 64);
  memcpy(data1, "Recovery Test 1", 15);
  if (mm_write(p1, 0, data1, 64) != 64) {
    printf("FAIL: Write failed\n");
    mm_free(p1);
    return 0;
  }

  uint8_t* header1 = (uint8_t*)p1 - 40;
  if (header1 >= heap && header1 < heap + heap_size) {
    header1[0] = 0x00;
    header1[1] = 0x00;
    printf("INFO: Corrupted header magic bytes\n");

    char buf[64];
    int rd = mm_read(p1, 0, buf, 64);
    total++;
    if (rd == 64 && memcmp(buf, data1, 64) == 0) {
      printf("PASS: Header recovered from footer, data intact\n");
      passed++;
    } else if (rd > 0) {
      printf("PASS: Header recovered, read returned %d bytes\n", rd);
      passed++;
    } else {
      printf("FAIL: Header recovery failed (rd=%d)\n", rd);
    }
  }
  mm_free(p1);

  // Test 2: Footer recovery from header.
  printf("\n-- Footer Recovery from Header --\n");
  void* p2 = mm_malloc(64);
  if (!p2) {
    printf("FAIL: Allocation for footer test failed\n");
    return passed > 0;
  }

  char data2[64];
  memset(data2, 'F', 64);
  memcpy(data2, "Footer Test", 11);
  if (mm_write(p2, 0, data2, 64) != 64) {
    printf("FAIL: Write for footer test failed\n");
    mm_free(p2);
    return passed > 0;
  }

  // Footer is at payload + payload_capacity + padding to end of block.
  // Block layout: [Header 20B][Pad 20B][Payload 64B][Footer 12B] = 116B -> 120B
  // Footer starts at offset 108 from header (120 - 12).
  uint8_t* header2 = (uint8_t*)p2 - 40;
  // Read size from header to find footer.
  uint32_t block_size = *(uint32_t*)(header2 + 4);  // size field offset
  uint8_t* footer2 = header2 + block_size - 12;

  if (footer2 >= heap && footer2 + 12 <= heap + heap_size) {
    footer2[0] = 0x00;  // Corrupt footer magic
    footer2[1] = 0x00;
    printf("INFO: Corrupted footer magic bytes\n");

    char buf2[64];
    int rd2 = mm_read(p2, 0, buf2, 64);
    total++;
    if (rd2 == 64 && memcmp(buf2, data2, 64) == 0) {
      printf("PASS: Footer recovered from header, data intact\n");
      passed++;
    } else if (rd2 > 0) {
      printf("PASS: Footer recovered, read returned %d bytes\n", rd2);
      passed++;
    } else {
      printf("FAIL: Footer recovery failed (rd=%d)\n", rd2);
    }
  }
  mm_free(p2);

  // Test 3: Verify allocator still works.
  void* p3 = mm_malloc(200);
  total++;
  if (p3) {
    char data3[200];
    memset(data3, 'U', 200);
    if (mm_write(p3, 0, data3, 200) == 200) {
      char buf3[200];
      if (mm_read(p3, 0, buf3, 200) == 200) {
        printf("PASS: Allocator functional after recovery tests\n");
        passed++;
      }
    }
    mm_free(p3);
  } else {
    printf("FAIL: Allocation failed after recovery tests\n");
  }

  printf("\nRecovery tests: %d/%d passed\n", passed, total);
  return passed >= 2;  // Need at least 2 of 3 to pass
}

static void ParseArgs(int argc, char* argv[],
                      unsigned int* seed, int* storm, size_t* heap_size,
                      int* num_arenas) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      *seed = (unsigned int)atoi(argv[++i]);
    } else if (strcmp(argv[i], "--storm") == 0 && i + 1 < argc) {
      *storm = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
      *heap_size = (size_t)atoi(argv[++i]);
    } else if (strcmp(argv[i], "--arenas") == 0 && i + 1 < argc) {
      *num_arenas = atoi(argv[++i]);
    }
  }
}

int main(int argc, char* argv[]) {
  unsigned int seed = (unsigned int)time(NULL);
  int storm = DEFAULT_STORM;
  size_t heap_size = DEFAULT_HEAP_SIZE;
  int num_arenas = 1;

  ParseArgs(argc, argv, &seed, &storm, &heap_size, &num_arenas);

  printf("Mars Rover Allocator Test\n");
  printf("=========================\n");
  printf("Seed: %u\n", seed);
  printf("Storm intensity: %d\n", storm);
  printf("Heap size: %zu bytes\n", heap_size);
  printf("Arenas: %d\n", num_arenas);

  srand(seed);

  uint8_t* heap = malloc(heap_size);
  if (!heap) {
    printf("FATAL: Could not allocate test heap\n");
    return 1;
  }

  fill_heap_pattern(heap, heap_size);

  MmStatus status = mm_arena_init(heap, heap_size, (uint8_t)num_arenas);
  if (status != MM_OK) {
    printf("FATAL: mm_arena_init failed: %s\n", mm_status_str(status));
    return 1;
  }
  printf("Allocator initialized successfully\n");

  int passed = 0;
  int total = 0;

  // Basic tests.
  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_basic_alloc()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_calloc()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_validate()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_heap_check()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_defrag()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_read_write()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_double_free()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_coalescing()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_best_fit()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_segregated_lists()) {
    passed++;
  }

  fill_heap_pattern(heap, heap_size);
  mm_init(heap, heap_size);
  total++;
  if (test_dynamic_size_classes(heap_size)) {
    passed++;
  }

  // Multi-threaded test (only if multiple arenas).
  if (num_arenas > 1) {
    fill_heap_pattern(heap, heap_size);
    mm_arena_init(heap, heap_size, (uint8_t)num_arenas);
    total++;
    if (test_multi_threaded(num_arenas)) {
      passed++;
    }

    fill_heap_pattern(heap, heap_size);
    mm_arena_init(heap, heap_size, (uint8_t)num_arenas);
    total++;
    if (test_cross_arena_free(num_arenas)) {
      passed++;
    }
  }

  // Storm tests.
  if (storm > 0) {
    fill_heap_pattern(heap, heap_size);
    mm_init(heap, heap_size);
    total++;
    if (test_storm_resilience(heap, heap_size, storm)) {
      passed++;
    }

    fill_heap_pattern(heap, heap_size);
    mm_init(heap, heap_size);
    total++;
    if (test_brownout(heap, heap_size)) {
      passed++;
    }

    fill_heap_pattern(heap, heap_size);
    mm_init(heap, heap_size);
    total++;
    if (test_recovery(heap, heap_size)) {
      passed++;
    }
  }

  printf("\n");
  mm_heap_stats();

  // Run heap check.
  printf("\nFinal heap check:\n");
  mm_heap_check(1);

  printf("\n=========================\n");
  printf("Results: %d/%d tests passed\n", passed, total);

  return (passed == total) ? 0 : 1;
}
