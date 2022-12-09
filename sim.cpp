#include "sim.h"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Vertex.hpp"
#include "SFML/Window/Keyboard.hpp"

#include "SFML/Window/Mouse.hpp"
#include "col_utils.h"
#include "collision.h"
#include "imgui.h"
#include "utils.h"
#include "soft_body.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <vector>
namespace EPI_NAMESPACE {
    void setupImGuiFont() {
        sf::Font consolas;
        if (!consolas.loadFromFile(CONSOLAS_PATH)) {
            std::cout << "error while lodaing font file!";
            exit(1);
        }

        //for bigger font
        ImGuiIO& io = ImGui::GetIO();
        io.Fonts->Clear();
        io.WantCaptureMouse = true;
        io.WantCaptureMouseUnlessPopupClose = true;
        
        ImFont* font = io.Fonts->AddFontFromFileTTF(CONSOLAS_PATH, 24.f);
        ImGui::SFML::UpdateFontTexture();
    }
    void Sim::setup() {

        aabb_inner.setSize(aabb_inner.size() - vec2f(padding * 2.f, padding * 2.f));
        //RigidBody model_poly = Polygon(vec2f(), 0.f, mini_model);

#define ADD_SIDE(ax, ay, bx, by)\
        {\
            std::vector<vec2f> model;\
            model.push_back(vec2f(aabb_inner.ax, aabb_inner.ay));\
            model.push_back(vec2f(aabb_inner.bx, aabb_inner.by));\
            model.push_back(vec2f(aabb_outer.bx, aabb_outer.by));\
            model.push_back(vec2f(aabb_outer.ax, aabb_outer.ay));\
            auto avg = std::reduce(model.begin(), model.end());\
            avg /= (float)model.size();\
            for(auto& m : model)\
                m -= avg;\
            auto t = RigidPolygon(avg, 0.f, model);\
            t.isStatic = true;\
            polys.push_back(std::make_unique<RigidPolygon>(t));\
        }

        ADD_SIDE(min.x, min.y, min.x, max.y);
        ADD_SIDE(max.x, min.y, min.x, min.y);
        ADD_SIDE(max.x, max.y, min.x, max.y);
        ADD_SIDE(max.x, max.y, max.x, min.y);

        //RigidPoly p0 = Polygon(vec2f(), 0.f, mini_model);
        for(const auto& d : polys)
            pm.bind(d.get());
        pm.steps = 3U;
    }
    void Sim::onEvent(const sf::Event &event) {
        if (event.type == sf::Event::Closed)
            window.close();
        if(event.type == sf::Event::MouseButtonPressed) {
            if(event.mouseButton.button == sf::Mouse::Button::Left) {
                last_mpos = (vec2f)sf::Mouse::getPosition(window);
                selected= nullptr;
                for(auto& p : polys) {
                    if(!p->isStatic && PointVPoly(last_mpos, *p)) {
                        selected = p.get();
                        p->isStatic = true;
                        break;
                    }
                }
                for(auto& c : circs) {
                    if(!c->isStatic && PointVCircle(last_mpos, *c)) {
                        selected = c.get();
                        c->isStatic = true;
                        break;
                    }
                }
                Node* closest_node = nullptr;
                float closest_len = INFINITY;
                /*
                for(auto& n : softy.getNodes()) {
                    float len = qlen(last_mpos - n->pos) - n->radius * n->radius;
                    if(len < closest_len) {
                        closest_len = len;
                        closest_node = n.get();
                    }
                }
                if(len(closest_node->pos - last_mpos) < 50.f) {
                    selected_node = closest_node;
                }
                */
            }
        }
        if(event.type == sf::Event::MouseButtonReleased) {
            if(event.mouseButton.button == sf::Mouse::Button::Left) {
                auto mpos = (vec2f)sf::Mouse::getPosition(window);
                auto dif = mpos - last_mpos;
                if(selected) {
                    selected->isStatic = false;
                    selected->vel += dif / 10.f;
                }else if(!selected_node) {
                    saved_point_vec.push_back(mpos);
                }
                selected = nullptr;
            }
            selected_node = nullptr;
        }
        if(event.type == sf::Event::KeyPressed) {
            if(event.key.code == sf::Keyboard::R) {
                saved_point_vec.clear();
            }
            bool curIsStatic = false;
            if(event.key.code == sf::Keyboard::S) {
                curIsStatic = true;
            }
            if(event.key.code == sf::Keyboard::S || event.key.code == sf::Keyboard::D) {
                if(saved_point_vec.size() >= 3) {
                    RigidPolygon t(PolygonPoints(saved_point_vec));
                    t.isStatic = curIsStatic;
                    t.mass = 1.f;
                    polys.push_back(std::make_unique<RigidPolygon>(t));
                    //std::cerr << getInertia(vec2f(0, 0), t.getModelVertecies(), t.mass) << "\n";
                    pm.bind(polys.back().get());
                }
                else {
                    RigidCircle t((vec2f)sf::Mouse::getPosition(window), 50.f);
                    t.isStatic = curIsStatic;
                    circs.push_back(std::make_unique<RigidCircle>(t));
                    pm.bind(circs.back().get());
                }
                saved_point_vec.clear();
            }
            if(event.key.code == sf::Keyboard::BackSpace && selected) {
                for(auto p = polys.begin(); p != polys.end(); p++) {
                    if((void*)p->get() == (void*)selected) {
                        polys.erase(p);
                        pm.unbind((RigidPolygon*)selected);
                        break;
                    }
                }
                for(auto c = circs.begin(); c != circs.end(); c++) {
                    if((void*)c->get() == (void*)selected) {
                        circs.erase(c);
                        pm.unbind((RigidCircle*)selected);
                        break;
                    }
                }

                selected = nullptr;
            }
        }
    }
    void Sim::update() {

        ImGui::Begin("Demo window");
        {
            static int tsteps = 2;
            ImGui::SliderInt("change step count" , &tsteps, 1, 50);
            pm.steps = tsteps;
        }
        ImGui::End();

        if(selected_node)
            selected_node->pos = (vec2f)sf::Mouse::getPosition(window);


        pm.update();
        window.clear(PastelColor::bg3);
        //drawing

        //drawFill(window, softy, PastelColor::blue);
        sf::CircleShape c(10.f);
        c.setPosition(softy.center() - vec2f(10.f, 10.f));
        c.setFillColor(PastelColor::orange);
        window.draw(c);
        //softy.update(pm.m_polys);
        for(auto& p : polys) {
            clr_t color = PastelColor::gray;
            if(!p->isStatic) {
                if(PointVPoly((vec2f)sf::Mouse::getPosition(window), *p))
                    color = PastelColor::red;
                else
                    color = PastelColor::aqua;
            }
            drawFill(window, *p, color);
        }
        for(auto& c : circs) {
            clr_t color = PastelColor::gray;
            if(!c->isStatic) {
                if(PointVCircle((vec2f)sf::Mouse::getPosition(window), *c))
                    color = PastelColor::red;
                else
                    color = PastelColor::aqua;
            }
            sf::CircleShape cs(c->radius);
            cs.setPosition(c->pos - vec2f(c->radius, c->radius));
            cs.setFillColor(color);
            window.draw(cs);
        }
        for(auto p : saved_point_vec) {
            float r = 5.f;
            sf::CircleShape c(r);
            c.setPosition(p - vec2f(r, r));
            c.setFillColor(clr_t::Magenta);
            window.draw(c);
        }
    }
    void Sim::Run() {
        sf::Clock DeltaClock;

        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event))
            {
                ImGui::SFML::ProcessEvent(event);
                onEvent(event);
            }
            ImGui::SFML::Update(window, DeltaClock.restart());

            update();

            ImGui::SFML::Render(window);
            window.display();
        }

    }
    Sim::Sim(float w, float h) : m_width(w), m_height(h), window(sf::VideoMode(w, h), "collisions") {
        window.setFramerateLimit(60);
        ImGui::SFML::Init(window);

        setupImGuiFont();
        softy = SoftBody(model_softbody, vec2i(10, 10), 1.f);
        /*
        softy.m_nodes[idx++]->isStatic = true;
        while(abs(last_y - softy.m_nodes[idx++]->pos.y) < 10.f) {
        }
        softy.m_nodes[idx - 2]->isStatic = true;
        */
        setup();

    }

}
