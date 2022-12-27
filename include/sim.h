#pragma once
#include "SFML/Window/Event.hpp"
#include "collision.h"
#include "imgui.h"

#include "types.hpp"
#include "col_utils.h"
#include "physics_manager.hpp"

namespace EPI_NAMESPACE {
    class Sim {
        sf::RenderWindow window;
    public:
        sf::Clock deltaClock;

        float m_width, m_height;
        PhysicsManager pm;
        std::vector<std::unique_ptr<RigidPolygon>> polys;
        std::vector<std::unique_ptr<RigidCircle>> circs;


        AABB aabb_outer = {{0, 0}, (vec2f)window.getSize()};
        float padding = 50.f;
        AABB aabb_inner = aabb_outer;

        std::vector<vec2f> polygon_creation_vec;
        vec2f last_mpos;

        bool isThrowing = false;
        Rigidbody* hovered_last = nullptr;
        Rigidbody* hovered_now = nullptr;
        //Node* selected_node = nullptr;
        float pin_angle = 0.f;
        vec2f pin_offset;

        float default_dynamic_radius = 50.f;


        void Run();
        virtual void update(float delT);
        virtual void setup();
        virtual void onEvent(const sf::Event& event, float delT);

        Sim(float w, float h);
    };

}
