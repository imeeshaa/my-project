#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR ((uint8_t*)0x20001000)
#define HEAP_SIZE       (4 * 1024)        // 4 KB heap
#define BLOCK_SIZE      16                // 16-byte blocks
#define BLOCK_COUNT     (HEAP_SIZE / BLOCK_SIZE)

static uint8_t block_map[BLOCK_COUNT];

void heap_init(void) {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        block_map[i] = 0; // mark all free
    }
}

void* heap_alloc(size_t size) {
    if (size == 0) return NULL;

    int blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    int free_count = 0;
    int start_index = -1;

    // Find first sequence of free blocks
    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (block_map[i] == 0) {
            if (free_count == 0) start_index = i;
            free_count++;
            if (free_count == blocks_needed) break;
        } else {
            free_count = 0;
            start_index = -1;
        }
    }

    // Not enough space
    if (start_index == -1) return NULL;

    // Mark blocks: store the number of blocks in the first one
    block_map[start_index] = blocks_needed;
    for (int i = start_index + 1; i < start_index + blocks_needed; i++) {
        block_map[i] = 0xFF; // part of same allocation
    }

    // Compute and return actual memory address
    uint8_t* address = HEAP_START_ADDR + (BLOCK_SIZE * start_index);
    return (void*)address;
}

void heap_free(void* ptr) {
    if (ptr == NULL) return;

    uint8_t* addr = (uint8_t*)ptr;

    // Sanity check: must be inside heap range
    if (addr < HEAP_START_ADDR || addr >= HEAP_START_ADDR + HEAP_SIZE)
        return;

    int index = (addr - HEAP_START_ADDR) / BLOCK_SIZE;
    int block_count = block_map[index];

    // If invalid (not a starting block), ignore
    if (block_count == 0 || block_count == 0xFF)
        return;

    // Free all blocks of this allocation
    for (int i = index; i < index + block_count && i < BLOCK_COUNT; i++) {
        block_map[i] = 0;
    }
}
