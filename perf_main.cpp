#include "SFML/System/Clock.hpp"
#include "sim.h"
#include "types.hpp"

using namespace epi;

int main() {
    {
        std::cout << "+====Poly Simulation====+\n";
        Sim s(2000.f, 2000.f, {}, 0, 
              PolygonReg(vec2f(1000.f, 1000.f), 0.f, 5U, 50.f), 50U, 3.f);
        s.Run();
    }
    {
        std::cout << "+====Circle Simulation====+\n";
        Sim s(2000.f, 2000.f, Circle(vec2f(500.f, 500.f), 7.f), 800U, {}, 0, 3.f);
        s.Run();
    }
    {
        std::cout << "+====Mixed Simulation====+\n";
        Sim s(2000.f, 2000.f, 
              Circle(vec2f(1000.f, 1000.f), 35.f), 100U, 
                  PolygonReg(vec2f(1000.f, 1000.f), 0.f, 5U, 35.f), 50U, 3.f);
        s.Run();
    }

    return 0;
}
