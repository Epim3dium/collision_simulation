#include "allocator.hpp"
#include "rigidbody.hpp"
#include "sim.hpp"

using namespace epi;

int main() {
    allocator::PoolAllocator allocator(64U, 10U);
    char* str = (char*)allocator.allocate();
    Sim s(2000.f, 2000.f);
    s.Run();

    return 0;
}
