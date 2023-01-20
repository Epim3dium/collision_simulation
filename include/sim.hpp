#pragma once
#include "SFML/Window/Event.hpp"
#include "rigidbody.hpp"
#include "solver.hpp"
#include "imgui.h"

#include "trigger.hpp"
#include "types.hpp"
#include "physics_manager.hpp"
#include "particle_manager.hpp"

#include <cstddef>
#include <memory>
#include <set>
#include <vector>

namespace EPI_NAMESPACE {


struct SelectingTrigger : public TriggerPolygonInterface {
    std::set<Rigidbody*> selected;
    void onActivation(Rigidbody* rb, vec2f cn) override;
    void clear() { selected.clear(); }
    void setShape(AABB aabb) {
        *this = PolygonfromAABB(aabb);
    }
    SelectingTrigger(const Polygon& shape) : TriggerPolygonInterface(shape) {}
};
class Sim {
    sf::RenderWindow window;
    float m_width, m_height;

    void delFromPolys(Rigidbody* rb) {
        for(auto p = polys.begin(); p != polys.end(); p++) {
            if((void*)*p == (void*)rb) {
                polys.erase(p);
                break;
            }
        }
    }
    void delFromCircs(Rigidbody* rb) {
        for(auto p = circs.begin(); p != circs.end(); p++) {
            if((void*)*p == (void*)rb) {
                circs.erase(p);
                break;
            }
        }
    }
public:
    std::vector<float> FPS;
    float total_sim_time = 0.f;
    float max_sim_time = INFINITY;

    sf::Clock deltaClock;

    PhysicsManager physics_manager;
    ParticleManager particle_manager = ParticleManager(8192U);
    
    
    std::vector<RigidPolygon*> polys;
    std::vector<RigidCircle*> circs;
    std::set<Rigidbody*> rigidbodies;

    void createRigidbody(const RigidPolygon& poly) {
        auto ptr = physics_manager.createRigidbody(poly);
        polys.push_back(ptr);
        rigidbodies.insert(ptr);
    }
    void createRigidbody(const RigidCircle& circ) {
        auto ptr = physics_manager.createRigidbody(circ);
        circs.push_back(ptr);
        rigidbodies.insert(ptr);
    }
    void removeRigidbody(Rigidbody* r) {
        delFromPolys(r);
        delFromCircs(r);
        rigidbodies.erase(r);
        physics_manager.removeRigidbody(r);
    }
    void crumbleSquarely(Rigidbody* poly);


    AABB aabb_outer = {{0, 0}, (vec2f)window.getSize()};
    float padding = 50.f;
    AABB aabb_inner = aabb_outer;

    std::vector<vec2f> polygon_creation_vec;

    struct {
        std::unique_ptr<SelectingTrigger> trigger;
        bool isMaking = false;
        bool isHolding = false;
        bool isThrowing = true;
        float making_time = 0.f;
        std::map<Rigidbody*, vec2f> offsets;
        std::set<Rigidbody*> selected;
        vec2f last_mouse_pos;
    }selection;
    //Node* selected_node = nullptr;
    float pin_angle = 0.f;
    vec2f pin_offset;

    float default_dynamic_radius = 50.f;


    void Run();
    virtual void update(float delT);
    virtual void draw();
    virtual void setup();
    virtual void onEvent(const sf::Event& event, float delT);

    Sim(float w, float h);
    ~Sim();
};

}
