#pragma once
#include "SFML/Window/Event.hpp"
#include "rigidbody.hpp"
#include "solver.hpp"
#include "imgui.h"

#include "trigger.hpp"
#include "types.hpp"
#include "col_utils.h"
#include "physics_manager.hpp"

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
        public:
            sf::Clock deltaClock;

            float m_width, m_height;
            PhysicsManager pm;
            std::vector<std::unique_ptr<RigidPolygon>> polys;
            std::vector<std::unique_ptr<RigidCircle>> circs;
            std::set<Rigidbody*> rigidbodies;
            void delFromPolys(Rigidbody* rb) {
                for(auto p = polys.begin(); p != polys.end(); p++) {
                    if((void*)p->get() == (void*)rb) {
                        polys.erase(p);
                        pm.unbind((RigidPolygon*)rb);
                        break;
                    }
                }
            }
            void delFromCircs(Rigidbody* rb) {
                for(auto p = circs.begin(); p != circs.end(); p++) {
                    if((void*)p->get() == (void*)rb) {
                        circs.erase(p);
                        pm.unbind((RigidCircle*)rb);
                        break;
                    }
                }
            }


            AABB aabb_outer = {{0, 0}, (vec2f)window.getSize()};
            float padding = 50.f;
            AABB aabb_inner = aabb_outer;

            std::vector<vec2f> polygon_creation_vec;

            struct {
                std::unique_ptr<SelectingTrigger> trigger;
                bool isMaking = false;
                float making_time = 0.f;
                bool isHolding = false;
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
            virtual void setup();
            virtual void onEvent(const sf::Event& event, float delT);

            Sim(float w, float h);
        };

}
