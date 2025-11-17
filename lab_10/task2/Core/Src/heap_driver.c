#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR ((uint8_t*)0x20001000)
#define HEAP_SIZE       (4 * 1024)        // 4 KB heap
#define BLOCK_SIZE      16                // 16-byte blocks
#define BLOCK_COUNT     (HEAP_SIZE / BLOCK_SIZE)

static uint8_t block[BLOCK_COUNT];

typedef struct{
    uint8_t blocks;
} header; 

void heap_init(){
    for (int i=0; i<BLOCK_COUNT; i++){
        block[i] = 0; //initializes as free
    }
}

void* heap_alloc(size_t size){
    
    size_t total = size + sizeof(header);
    int n = (total+ BLOCK_SIZE -1) / BLOCK_SIZE;
    int count = 0;
    int index = -1;
    
    if(n>BLOCK_COUNT) return NULL;

    
    for (int i=0; i<BLOCK_COUNT; i++){
        if (!block[i]){ 
            count++;
            if(count==n){
                index = (i-n)+1;
                break;
            }}
        else 
            count = 0;
    }
    
    if (index < 0)
        return NULL;
    
    for(int i=index; i<index+n;  i++){
        block[i] = 1;
    }
    
    header* t = (header*)(HEAP_START_ADDR+index*BLOCK_SIZE);
    t->blocks = n;
    
    return t+1;
    
}


void heap_free(void* ptr){
    if (ptr == NULL)
        return;
    
    header* t = (header*)((uint8_t*)ptr - sizeof(header)); //header
    int b = t->blocks; //blocks stored in header
    int s = ((uint8_t*)t - HEAP_START_ADDR)/BLOCK_SIZE; //starting address
    
    for (int i=s; i<b+s; i++){
        block[i] = 0; //marks them free
    }
}