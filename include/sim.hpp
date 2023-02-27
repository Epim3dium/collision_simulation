#pragma once
#include "SFML/Window/Event.hpp"
#include "restraint.hpp"
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
        *this = Polygon::CreateFromAABB(aabb);
    }
    SelectingTrigger(const Polygon& shape) : TriggerPolygonInterface(shape) {}
};
class Sim {
    sf::RenderWindow window;
    float m_width, m_height;

    void delFromPolys(Rigidbody* rb) {
        for(auto p = polys.begin(); p != polys.end(); p++) {
            if(p->get() == rb) {
                polys.erase(p);
                break;
            }
        }
    }
    void delFromCircs(Rigidbody* rb) {
        for(auto p = circs.begin(); p != circs.end(); p++) {
            if(p->get() == rb) {
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
    float gravity;
    ParticleManager particle_manager = ParticleManager(8192U);
    
    
    std::vector<std::shared_ptr<RigidPolygon>> polys;
    std::vector<std::shared_ptr<RigidCircle>> circs;
    std::set<Rigidbody*> rigidbodies;
    std::vector<std::shared_ptr<RestraintInterface>> restraints;

    void createRigidbody(const RigidPolygon& poly) {
        polys.push_back(std::make_shared<RigidPolygon>(poly));
        rigidbodies.insert(polys.back().get());
        physics_manager.bind(std::dynamic_pointer_cast<Rigidbody>(polys.back()));
    }
    void createRigidbody(const RigidCircle& circ) {
        circs.push_back(std::make_shared<RigidCircle>(circ));
        rigidbodies.insert(circs.back().get());
        physics_manager.bind(std::dynamic_pointer_cast<Rigidbody>(circs.back()));
    }
    void removeRigidbody(Rigidbody* r) {
        delFromPolys(r);
        delFromCircs(r);
        rigidbodies.erase(r);
    }
    void crumbleSquarely(Rigidbody* poly);


    AABB aabb_outer = {{0, 0}, (vec2f)window.getSize()};
    float padding = 50.f;
    AABB aabb_inner = aabb_outer;

    std::vector<vec2f> polygon_creation_vec;

    struct {
        std::shared_ptr<SelectingTrigger> trigger;
        bool isMaking = false;
        bool isHolding = false;
        bool isThrowing = true;
        float making_time = 0.f;
        std::map<Rigidbody*, vec2f> offsets;
        std::set<Rigidbody*> selected;
        Rigidbody* last_restrain_sel = nullptr;
        vec2f last_restrain_sel_off;

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
