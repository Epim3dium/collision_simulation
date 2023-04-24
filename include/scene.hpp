#ifndef SCENE_H
#define SCENE_H
#include "SFML/System/Clock.hpp"
#include "SFML/System/Time.hpp"
#include "game_object.hpp"
#include "types.hpp"
#include "io_manager.hpp"
#include "physics_manager.hpp"

#include <cstddef>
#include <exception>
#include <memory>
#include <set>
#include <vector>

namespace epi {
class Scene : public GameObject {
    struct BailException : public std::exception {
        std::string message;
        const char* what() const noexcept override {
            return "BailException";
        }
        BailException(std::string msg = "") : message(msg) {}
    };

    sf::Clock deltaTimer;
protected:
    inline void bail(std::string msg = "") {
        throw BailException(msg);
    }
public:
    #define SCENE_TYPE (typeid(Scene).hash_code())
    Property getPropertyList() const override {
        return {SCENE_TYPE, "Scene"};
    }
    virtual void update(sf::Time delT) {}
    virtual int setup() { return 0; }

    Scene() {}
    ~Scene() {
        notify(*this, "destroyed");
    }
    friend void run(Scene& scene);
};

class DefaultScene : public Scene, Signal::Observer {
    vec2i _size;
protected:
    IOManager* _io_manager;
    PhysicsManager* _physics_manager;
    AABB sim_window;

    virtual void onNotify(const GameObject& obj, Signal::Event event) override = 0;

    virtual void onUpdate(float delT) {
    }
    virtual void onRender(sf::RenderTarget& target) {
    }
    virtual void onSetup() {
    }
public:
    void update(sf::Time delT) override {
        _io_manager->pollEvents();
        ImGui::SFML::Update(_io_manager->getWindow(), _io_manager->getRenderObject(), delT);
        onUpdate(delT.asSeconds());
        onRender(_io_manager->getRenderObject());
        _io_manager->display();
    }
    int setup() override { 
        _io_manager = this->addComponent(new IOManager(_size, "demo window"));
        auto tmp = sim_window;
        tmp.setSize(tmp.size() * 2.f);
        _physics_manager = this->addComponent(new PhysicsManager(tmp));
        _io_manager->addObserver(this);
        onSetup();
        return 0; 
    }
    DefaultScene(vec2i s = {2000, 2000}) : _size(s), sim_window({{0, 0}, (vec2f)s}) {}
};


}
#endif
