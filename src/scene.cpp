#include "scene.hpp"
#include <exception>
#include <set>

namespace epi {
void run(Scene& scene) {
    int err = scene.setup();
    if(err) {
        std::cout << "error on setup: " << err << "\n";
    }
    try {
        while(scene.isActive) {
            scene.update(scene.deltaTimer.restart());
        }
    }
    catch(Scene::BailException& e) {
        std::cout << e.what() << ", " << e.message << "\n";
    }
    catch(std::exception& e) {
        std::cout << e.what() << "\n";
    }
}
}
