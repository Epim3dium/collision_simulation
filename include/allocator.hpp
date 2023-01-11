#pragma once
#include <cstddef>
#include <functional>
#include <stdlib.h>
#include <vector>
#include <stack>
namespace epi {
namespace allocator {

struct Chunk {
    /**
   * When a chunk is free, the `next` contains the
   * address of the next chunk in a list.
   *
   * When it's allocated, this space is used by
   * the user.
   */
    Chunk *next;
};


class PoolBlockAllocator {
public:
PoolBlockAllocator(size_t chunksPerBlock)
    : mChunksPerBlock(chunksPerBlock) {}
    void *allocate(size_t size);
    void deallocate(void *ptr, size_t size);

private:
/**
   * Number of chunks per larger block.
   */
    size_t mChunksPerBlock;

/**
   * Allocation pointer.
   */
    Chunk *mAlloc = nullptr;

/**
   * Allocates a larger block (pool) for chunks.
   */
    Chunk *allocateBlock(size_t);
};
class PoolAllocator {
public:
PoolAllocator(size_t maxObjCount, size_t objSize)
    : mMaxObjCount(maxObjCount), mObjSize(objSize)
    {
        mRoot = malloc(maxObjCount * mObjSize);
        mFreelist.reserve(maxObjCount);
        mFreelist.push_back({mRoot, maxObjCount * mObjSize});
    }
    void *allocate();
    void deallocate(void *ptr);

    void forEach(std::function<void(void*)> job);
    template<class T>
    void forEach(std::function<void(T*)> job) {
        forEach(job);
    }
    void* getRoot() const { return mRoot; }
private:
    struct MemInfo {
        void* ptr;
        size_t size;
    };

    size_t mMaxObjCount;
    size_t mObjSize;
    std::vector<MemInfo> mFreelist;
    void* mRoot;

};

}
}
