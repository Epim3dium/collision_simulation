#include "allocator.hpp"
#include <malloc/_malloc.h>
#include <iostream>

namespace epi {
namespace allocator {
Chunk *PoolBlockAllocator::allocateBlock(size_t chunkSize) {
    size_t blockSize = mChunksPerBlock * chunkSize;

    // The first chunk of the new block.
    Chunk *blockBegin = reinterpret_cast<Chunk *>(malloc(blockSize));

    // Once the block is allocated, we need to chain all
    // the chunks in this block:

    Chunk *chunk = blockBegin;

    for (int i = 0; i < mChunksPerBlock - 1; ++i) {
    chunk->next =
    reinterpret_cast<Chunk *>(reinterpret_cast<char *>(chunk) + chunkSize);
    chunk = chunk->next;
    }

    chunk->next = nullptr;

    return blockBegin;
}
/**
 * Returns the first free chunk in the block.
 *
 * If there are no chunks left in the block,
 * allocates a new block.
 */
void *PoolBlockAllocator::allocate(size_t size) {
    std::cerr << "memory allocated of size: " << size << "\n";

    // No chunks left in the current block, or no any block
    // exists yet. Allocate a new one, passing the chunk size:

    if (mAlloc == nullptr) {
    mAlloc = allocateBlock(size);
    }

    // The return value is the current position of
    // the allocation pointer:

    Chunk *freeChunk = mAlloc;

    // Advance (bump) the allocation pointer to the next chunk.
    //
    // When no chunks left, the `mAlloc` will be set to `nullptr`, and
    // this will cause allocation of a new block on the next request:

    mAlloc = mAlloc->next;

    return freeChunk;
}
void PoolAllocator::deallocate(void *ptr) {
    mFreelist.push_back({ptr, mObjSize});
}
void *PoolAllocator::allocate() {
    assert(mFreelist.size() != 0);
    auto& free = mFreelist.back();
    char* ptr = (char*)free.ptr;


    free.size -= mObjSize;
    if(free.size <= 0)
        mFreelist.pop_back();
    else
        free.ptr = reinterpret_cast<char*>(free.ptr) + mObjSize;
    //first byte is used to check if it is taken
    return ptr;
}
void PoolAllocator::forEach(std::function<void(void*)> job) {
    for(char* ptr = (char*)mRoot; ptr < (char*)mFreelist.back().ptr; ptr += mObjSize) {
        job(ptr);
    }
}

}
}
