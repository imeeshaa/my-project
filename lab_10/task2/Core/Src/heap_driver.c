#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define HEAP_START_ADDR ((uint8_t*)0x20001000)
#define HEAP_SIZE (4 * 1024)
#define BLOCK_SIZE 16
#define BLOCK_COUNT (HEAP_SIZE / BLOCK_SIZE)

uint8_t block_map[BLOCK_COUNT];

void heap_init(){

    for (int i =0; i<BLOCK_COUNT; i++){
        block_map[i] = 0;
    }}

void* heap_alloc(size_t size){
    if (size == 0){
        return NULL;
    }
   
    int j = (size+BLOCK_SIZE-1)/BLOCK_SIZE;
    int count = 0;
    int index = -1;
    for (int i = 0; i<BLOCK_COUNT; i++){
        
        if (block_map[i]==0){
            if (count == 0){
                index = i;}
            count++;
            if (count ==j){
            break;}}
        else{
            count = 0;
            index = -1;
        }
        }
    if (index == -1){
        return NULL;
    }

    for(int i=index; i<index+j; i++){
        block_map[i] = 1; 
    }
    uint8_t* address = HEAP_START_ADDR + (16*index);
    return (void*)address;

}

void heap_free(void* ptr){

if(ptr==NULL)
    return;

uint8_t* add = (uint8_t*)ptr;
uint8_t* header = add - BLOCK_SIZE; 
int block_count = block_map[(header-HEAP_START_ADDR)/BLOCK_SIZE]; 

int index =  (add-HEAP_START_ADDR)/16;

if(add<HEAP_START_ADDR || add>HEAP_START_ADDR+HEAP_SIZE)
return;

for (int i= index; i<index+block_count; i++){
    block_map[i]=0; //freed
}
}