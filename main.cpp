#include "sim.h"
#include <ostream>

using namespace epi;
int main() {
    Sim s(2000.f, 2000.f);
    s.Run();

    return 0;
}
