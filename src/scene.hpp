#ifndef SCENE_H
#define SCENE_H
#include "SFML/System/Clock.hpp"
#include "SFML/System/Time.hpp"
#include "types.hpp"
#include "io_manager.hpp"
#include "physics_manager.hpp"

#include <cstddef>
#include <exception>
#include <memory>
#include <set>
#include <vector>

namespace epi {
class Scene : Signal::Observer<IOManagerEvent> {
    struct BailException : public std::exception {
        std::string message;
        const char* what() const noexcept override {
            return "BailException";
        }
        BailException(std::string msg = "") : message(msg) {}
    };

    sf::Clock deltaTimer;
protected:
    IOManager io_manager;
    bool isActive = true;

    inline void bail(std::string msg = "") {
        throw BailException(msg);
    }
public:

    virtual void update(sf::Time delT) {}
    virtual int setup() { return 0; }

    Scene(vec2i s = {2000, 2000}) : io_manager(s) {
        io_manager.addObserver(this);
    }
    ~Scene() {
    }
    friend void run(Scene& scene);
};

class DefaultScene : public Scene {
    vec2i _size;
protected:
    AABB sim_window;
    PhysicsManager physics_manager;

    virtual void onUpdate(float delT) {
    }
    virtual void onRender(sf::RenderTarget& target) {
    }
    virtual void onSetup() {
    }
public:
    void update(sf::Time delT) override {
        io_manager.pollEvents();
        ImGui::SFML::Update(io_manager.getWindow(), io_manager.getRenderObject(), delT);
        auto delTsec = std::clamp(delT.asSeconds(), 0.f, 1.f);
        onUpdate(delTsec);
        onRender(io_manager.getRenderObject());
        io_manager.display();
    }
    int setup() override { 
        onSetup();
        return 0; 
    }
    DefaultScene(vec2i s = {2000, 2000}) : _size(s), sim_window({{0, 0}, (vec2f)s}), physics_manager(sim_window), Scene(s) {}
};


}
#endif
