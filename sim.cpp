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
        pm.bounciness_select = eSelectMode::Max;
        pm.friction_select = eSelectMode::Max;

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
            if(!hovered_now) {
                polygon_creation_vec.push_back((vec2f)sf::Mouse::getPosition(window));
            }else {
                if(hovered_now) {
                    isThrowing = true;
                    last_mpos = (vec2f)sf::Mouse::getPosition(window);
                }
            }
        }
        if(event.type == sf::Event::MouseButtonReleased) {
            if(event.mouseButton.button == sf::Mouse::Button::Left) {
                if(isThrowing) {
                    auto mpos = (vec2f)sf::Mouse::getPosition(window);
                    auto dif = mpos - last_mpos;
                    if(hovered_last) {
                        hovered_last->vel += dif / 10.f;
                    }else if(!selected_node) {
                        polygon_creation_vec.push_back(mpos);
                    }
                    isThrowing = false;
                }
            }
            selected_node = nullptr;
        }
        if(event.type == sf::Event::KeyPressed) {
            if(event.key.code == sf::Keyboard::R) {
                polygon_creation_vec.clear();
            }
            bool curIsStatic = false;
            if(event.key.code == sf::Keyboard::S) {
                curIsStatic = true;
            }
            if((event.key.code == sf::Keyboard::S || event.key.code == sf::Keyboard::D) && !hovered_now) {
                if(polygon_creation_vec.size() >= 3) {
                    RigidPolygon t(PolygonPoints(polygon_creation_vec));
                    t.isStatic = curIsStatic;
                    t.mass = 1.f;
                    polys.push_back(std::make_unique<RigidPolygon>(t));
                    //std::cerr << getInertia(vec2f(0, 0), t.getModelVertecies(), t.mass) << "\n";
                    pm.bind(polys.back().get());
                }
                else {
                    RigidCircle t((vec2f)sf::Mouse::getPosition(window), default_dynamic_radius);
                    t.isStatic = curIsStatic;
                    circs.push_back(std::make_unique<RigidCircle>(t));
                    pm.bind(circs.back().get());
                }
                polygon_creation_vec.clear();
            }else if(event.key.code == sf::Keyboard::D && hovered_now) {
                hovered_now->isStatic = false;
            }else if(event.key.code == sf::Keyboard::S && hovered_now) {
                hovered_now->isStatic = true;
            } else if(event.key.code == sf::Keyboard::BackSpace && hovered_now) {
                for(auto p = polys.begin(); p != polys.end(); p++) {
                    if((void*)p->get() == (void*)hovered_now) {
                        polys.erase(p);
                        pm.unbind((RigidPolygon*)hovered_now);
                        break;
                    }
                }
                for(auto c = circs.begin(); c != circs.end(); c++) {
                    if((void*)c->get() == (void*)hovered_now) {
                        circs.erase(c);
                        pm.unbind((RigidCircle*)hovered_now);
                        break;
                    }
                }
                hovered_now = nullptr;
                hovered_last = nullptr;
            }
        }
    }
    void Sim::update() {
        if(!isThrowing)
        {
            auto mpos = (vec2f)sf::Mouse::getPosition(window);
            Rigidbody* found = nullptr;
            for(auto& p : polys) {
                if(PointVPoly(mpos, *p)) {
                    found = p.get();
                    break;
                }
            }
            for(auto& c : circs) {
                if(PointVCircle(mpos, *c)) {
                    found = c.get();
                    break;
                }
            }
            hovered_now = found;
            if(found) {
                hovered_last = found;
            }
        }

        ImGui::Begin("Demo window");
        {
            static int tab_open = 0;
            ImGui::BeginTabBar("Settings");
            {
                static bool open_global = true;
                if(ImGui::BeginTabItem("global settings", &open_global))
                {
                    static int tsteps = 2;
                    ImGui::SliderInt("change step count" , &tsteps, 1, 50);
                    pm.steps = tsteps;
                    ImGui::SliderFloat("change gravity" , &pm.grav, -1.5f, 1.5f);
                    ImGui::SliderFloat("radius" , &default_dynamic_radius, 10.f, 50.f);
                    const char* select_modes[] = { "Min", "Max", "Avg" };
                    {
                        static int cur_choice_friction = 0;
                        ImGui::ListBox("choose mode friction", &cur_choice_friction, select_modes, 3);
                        pm.friction_select = (eSelectMode)cur_choice_friction;
                    }
                    {
                        static int cur_choice_bounce = 0;
                        ImGui::ListBox("choose mode bounce", &cur_choice_bounce, select_modes, 3);
                        pm.bounciness_select = (eSelectMode)cur_choice_bounce;
                    }ImGui::EndTabItem();
                } 
                static bool open_object = true;
                if(ImGui::BeginTabItem("object settings", &open_object))
                {
                    if(hovered_last) {
                        if(ImGui::Button("isStatic")) {
                            hovered_last->isStatic = !hovered_last->isStatic;
                        }
                        ImGui::Text("properties of selected object");
                        ImGui::SliderFloat("change mass" , &hovered_last->mass, 1.0f, 100.f);
                        ImGui::SliderFloat("change air_drag" , &hovered_last->mat.air_drag, 0.0f, 1.f);
                        ImGui::SliderFloat("change static fric" , &hovered_last->mat.sfriction, 0.0f, 1.f);
                        ImGui::SliderFloat("change dynamic fric" , &hovered_last->mat.dfriction, 0.0f, 1.f);
                        ImGui::SliderFloat("change restitution" , &hovered_last->mat.restitution, 0.0f, 1.f);
                    }else {
                        ImGui::Text("NONE SELECTED");
                    }ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        ImGui::End();

        if(selected_node)
            selected_node->pos = (vec2f)sf::Mouse::getPosition(window);


        pm.update();
        window.clear(PastelColor::bg4);
        //drawing

        //drawFill(window, softy, PastelColor::blue);
        sf::CircleShape c(10.f);
        c.setPosition(softy.center() - vec2f(10.f, 10.f));
        c.setFillColor(PastelColor::Orange);
        window.draw(c);
        //softy.update(pm.m_polys);
        for(auto& p : polys) {
            clr_t color = PastelColor::bg1;
            if(PointVPoly((vec2f)sf::Mouse::getPosition(window), *p)) {
                color = PastelColor::Red;
            } else {
                if(!p->isStatic) {
                    color = PastelColor::Aqua;
                }
            }
            drawFill(window, *p, color);
        }
        for(auto& c : circs) {
            clr_t color = PastelColor::Gray;
            if(PointVCircle((vec2f)sf::Mouse::getPosition(window), *c)) {
                color = PastelColor::Red;
            } else {
                if(!c->isStatic) {
                    color = PastelColor::Aqua;
                }
            }
            sf::CircleShape cs(c->radius);
            cs.setPosition(c->pos - vec2f(c->radius, c->radius));
            cs.setFillColor(color);
            window.draw(cs);
            float r = 5.f;
            cs.setRadius(r);
            cs.setPosition(c->pos + vec2f(cos(c->rot), sin(c->rot)) * c->radius / 2.f - vec2f(r, r));
            cs.setFillColor(PastelColor::bg3);
            window.draw(cs);
        }
        for(auto p : polygon_creation_vec) {
            float r = 5.f;
            sf::CircleShape c(r);
            c.setPosition(p - vec2f(r, r));
            c.setFillColor(clr_t::Magenta);
            window.draw(c);
        }
        for(auto cp : g_cps) {
            float r = 5.f;
            sf::CircleShape c(r);
            c.setPosition(cp - vec2f(r, r));
            c.setFillColor(PastelColor::Purple);
            window.draw(c);
        }
        g_cps.clear();
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
