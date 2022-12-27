#include "sim.h"
#include <ostream>

using namespace epi;
int main() {
    Sim s(1000.f, 1000.f);
    s.Run();

    return 0;
}
