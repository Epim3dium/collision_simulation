#include "sim.h"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Drawable.hpp"
#include "SFML/Graphics/RectangleShape.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Shape.hpp"
#include "SFML/Graphics/Vertex.hpp"
#include "SFML/Window/Keyboard.hpp"
#include "SFML/Window/Mouse.hpp"

#include "col_utils.h"
#include "imgui-SFML.h"
#include "imgui.h"
#include "restraint.hpp"
#include "rigidbody.hpp"
#include "types.hpp"
#include "particle.hpp"
#include "quad_tree.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <sys/_types/_size_t.h>
#include <vector>

namespace EPI_NAMESPACE {
void SelectingTrigger::onActivation(Rigidbody* rb, vec2f cn) {
    if(selected.contains(rb))
        return;
    selected.emplace(rb);
}
static bool PointVRigidbody(vec2f p, Rigidbody* rb) {
    switch(rb->getType()) {
        case eRigidShape::Circle:
            return PointVCircle(p, *(RigidCircle*)rb);
        break;
        case eRigidShape::Polygon:
            return PointVPoly(p, *(RigidPolygon*)rb);
        break;
    }
}
static void DrawRigidbody(Rigidbody* rb, const std::set<Rigidbody*> t, sf::RenderWindow& rw) {
    Color color = PastelColor::bg1;
    if(!rb->isStatic) {
        color = PastelColor::Aqua;
        if(rb->collider.isDormant)
            color = PastelColor::Green;
    }
    if(t.contains(rb))
        color = PastelColor::Red;
    switch(rb->getType()) {
        case eRigidShape::Circle: {
            RigidCircle c(*(RigidCircle*)rb);
            sf::CircleShape cs(c.radius);
            cs.setPosition(c.pos - vec2f(c.radius, c.radius));
            cs.setFillColor(color);
            rw.draw(cs);
            float r = c.radius / 2.f;
            cs.setRadius(r);
            cs.setPosition(c.pos + vec2f(cos(c.rot), sin(c.rot)) * c.radius / 2.f - vec2f(r, r));
            color.r /= 1.2f;
            color.g /= 1.2f;
            color.b /= 1.2f;
            cs.setFillColor(color);
            rw.draw(cs);
        }break;
        case eRigidShape::Polygon:
            drawFill(rw, *(RigidPolygon*)rb, color);
        break;
    }

}

static void setupImGuiFont() {
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
    physics_manager.segment_size = 50.f;
    physics_manager.steps = 10;
    physics_manager.grav = 1000.f;
    physics_manager.bounciness_select = eSelectMode::Max;
    physics_manager.friction_select = eSelectMode::Max;
    //pm.bind(new BasicSolver());
    /*
    std::vector<vec2f> m = {vec2f(0.f, 50.f), vec2f(25.f, 0.f), vec2f(0.f, -50.f), vec2f(-25.f, 0.f)};
    auto tmp = RigidPolygon(vec2f(500.f, 100.f), 0.f, m);

    polys.push_back(std::make_unique<RigidPolygon>( tmp ) );
    pm.bind(polys.back().get());
    auto a = polys.back().get();
    auto layer = a->collider.layer;
    for(int i = 0; i < 10; i++) {
        tmp.collider.layer += 1000;
        tmp.setPos(tmp.getPos() + vec2f(0.f, 50.f));
        polys.push_back(std::make_unique<RigidPolygon>( tmp ) );
        pm.bind(polys.back().get());
        auto b = polys.back().get();

        pm.bind(new RestraintPoint(2.f, a, 0U, b, 2U));
        a = b;
    }
    */

    aabb_inner.setSize(aabb_inner.size() - vec2f(padding * 2.f, padding * 2.f));
    {
        AABB trigger = aabb_inner;
        trigger.setCenter({10000.f, 10000.f});
        selection.trigger = std::make_unique<SelectingTrigger>(SelectingTrigger(PolygonfromAABB(trigger)));
        physics_manager.bind(selection.trigger.get());
    }
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
        rigidbodies.emplace(polys.back().get());\
    }

    ADD_SIDE(min.x, min.y, min.x, max.y);
    ADD_SIDE(max.x, min.y, min.x, min.y);
    ADD_SIDE(max.x, max.y, min.x, max.y);
    ADD_SIDE(max.x, max.y, max.x, min.y);

    //RigidPoly p0 = Polygon(vec2f(), 0.f, mini_model);
    for(const auto& d : polys)
        physics_manager.bind(d.get());
    physics_manager.steps = 3U;
}
void Sim::onEvent(const sf::Event &event, float delT) {
    if (event.type == sf::Event::Closed)
        window.close();
    else if(event.type == sf::Event::MouseButtonPressed) {
        if(event.mouseButton.button == sf::Mouse::Button::Left) {
            selection.last_mouse_pos = (vec2f)sf::Mouse::getPosition(window);
            for(auto r : rigidbodies) {
                if(PointVRigidbody(selection.last_mouse_pos, r)) {
                    if(!selection.selected.contains(r)) {
                        selection.selected = {r};
                    }
                    selection.isHolding = true;
                    break;
                }
            }
            if(selection.isHolding) {
                for(auto s : selection.selected) {
                    selection.offsets[s] = s->getPos() - selection.last_mouse_pos;
                    s->collider.lockRotation = true;
                }
            }else {
                selection.isMaking = true;
            }
        }
    }
    else if(event.type == sf::Event::MouseButtonReleased) {
        if(event.mouseButton.button == sf::Mouse::Button::Left) {
            if(selection.isHolding) {
                for(auto s : selection.selected) {
                    s->collider.lockRotation = false;
                }
            }
            else if(selection.making_time < 0.25f) {
                polygon_creation_vec.push_back(selection.last_mouse_pos);
            }
            selection.isMaking = false;
            selection.making_time = 0;
            selection.isHolding = false;

            selection.trigger->setShape({});
        }
    }
    else if(event.type == sf::Event::KeyPressed) {
        if(event.key.code == sf::Keyboard::Enter && polygon_creation_vec.size() > 2) {
            RigidPolygon t(PolygonfromPoints(polygon_creation_vec));
            polys.push_back(std::make_unique<RigidPolygon>(t));
            rigidbodies.emplace(polys.back().get());
            physics_manager.bind(polys.back().get());
            polygon_creation_vec.clear();
        }else if(event.key.code == sf::Keyboard::R) {
            selection.selected.clear();
            polygon_creation_vec.clear();
        }
        
    }
}
void Sim::update(float delT) {
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
        polygon_creation_vec.clear();
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::C)) {
        RigidCircle t((vec2f)sf::Mouse::getPosition(window), default_dynamic_radius);
        circs.push_back(std::make_unique<RigidCircle>(t));
        rigidbodies.emplace(circs.back().get());
        physics_manager.bind(circs.back().get());
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::V)) {
        vec2f mpos = (vec2f)sf::Mouse::getPosition(window);
        RigidPolygon t = PolygonReg(mpos, 3.141 / 4.f, 4U, default_dynamic_radius * sqrt(2.f));
        polys.push_back(std::make_unique<RigidPolygon>(t));
        rigidbodies.emplace(polys.back().get());
        physics_manager.bind(polys.back().get());
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
        vec2f mpos = (vec2f)sf::Mouse::getPosition(window);
        particle_manager.emit(30000U * delT, 
            Particle::PosInit(mpos),
            Particle::LifetimeInit(0.1f, 0.5f),
            Particle::ColorVecInit({PastelColor::Red}),
            Particle::ShapeInit(5.f),
            Particle::VelMagInit(250.f, 1000.f),
            Particle::VelAngleInit(0.f, 3.141f),
            Particle::ColorFuncInit(
                [](Color clr, float time)->Color {
                    Color result;
                    result.r = clr.r * time + PastelColor::Yellow.r * (1.f - time);
                    result.g = clr.g * time + PastelColor::Yellow.g * (1.f - time);
                    result.b = clr.b * time + PastelColor::Yellow.b * (1.f - time);
                    return result;
                })
        );
    }
    if(selection.isHolding) {
        for(auto s : selection.selected) {
            auto d = (vec2f)sf::Mouse::getPosition(window) + selection.offsets[s] - s->getPos();
            d /= sqrt(delT);
            s->velocity = d;
            s->setPos((vec2f)sf::Mouse::getPosition(window) + selection.offsets[s]);
        }
    }
    
    if(selection.isMaking) {
        selection.making_time += delT;
        selection.selected = selection.trigger->selected;
        selection.trigger->clear();
        AABB selection_aabb;
        auto mpos = (vec2f)sf::Mouse::getPosition(window);
        selection_aabb.min = vec2f(std::min(mpos.x, selection.last_mouse_pos.x), std::min(mpos.y, selection.last_mouse_pos.y));
        selection_aabb.max = vec2f(std::max(mpos.x, selection.last_mouse_pos.x), std::max(mpos.y, selection.last_mouse_pos.y));
        selection.trigger->setShape(selection_aabb);
    }
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
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::BackSpace)) {
        for(auto s : selection.selected) {
            delFromCircs(s);
            delFromPolys(s);
            rigidbodies.erase(s);
        }
        selection.selected.clear();
    }

    ImGui::Begin("Demo window");
    {
        static int tab_open = 0;
        ImGui::BeginTabBar("Settings");
        {
            static bool open_global = true;
            if(ImGui::BeginTabItem("global settings", &open_global))
            {
                ImGui::SliderFloat("change seg size" , &physics_manager.segment_size, 5.0f, 300.0f, "%.0f");
                static int tsteps = 10;
                ImGui::SliderInt("change step count" , &tsteps, 1, 50);
                physics_manager.steps = tsteps;
                ImGui::SliderFloat("change gravity" , &physics_manager.grav, -3000.f, 3000.f, "%.1f");
                ImGui::SliderFloat("radius" , &default_dynamic_radius, 5.f, 50.f);
                const char* select_modes[] = { "Min", "Max", "Avg" };
                {
                    static int cur_choice_friction = 0;
                    ImGui::ListBox("choose mode friction", &cur_choice_friction, select_modes, 3);
                    physics_manager.friction_select = (eSelectMode)cur_choice_friction;
                }
                {
                    static int cur_choice_bounce = 0;
                    ImGui::ListBox("choose mode bounce", &cur_choice_bounce, select_modes, 3);
                    physics_manager.bounciness_select = (eSelectMode)cur_choice_bounce;
                }ImGui::EndTabItem();
            } 
            static bool open_object = true;
            if(ImGui::BeginTabItem("object settings", &open_object) && selection.selected.size() != 0)
            {
                auto hovered_last = *selection.selected.begin();
                if(hovered_last) {
                    if(ImGui::Button("isStatic")) {
                        hovered_last->isStatic = !hovered_last->isStatic;
                    }
                    if(ImGui::Button("LockRotation")) {
                        hovered_last->collider.lockRotation = !hovered_last->collider.lockRotation;
                    }
                    ImGui::Text("properties of selected object");
                    ImGui::SliderFloat("change mass" , &hovered_last->mass, 1.0f, 100.f);
                    ImGui::SliderFloat("change air_drag" , &hovered_last->material.air_drag, 0.0f, 0.01f, "%.5f");
                    ImGui::SliderFloat("change static fric" , &hovered_last->material.sfriction, 0.0f, 1.f);
                    ImGui::SliderFloat("change dynamic fric" , &hovered_last->material.dfriction, 0.0f, 1.f);
                    ImGui::SliderFloat("change restitution" , &hovered_last->material.restitution, 0.0f, 1.f);
                    ImGui::InputScalar("layer: ", ImGuiDataType_U32, &hovered_last->collider.layer);
                }else {
                    ImGui::Text("NONE SELECTED");
                }ImGui::EndTabItem();
            }
            ImGui::Text("total bodies: %d", int(rigidbodies.size()));
            ImGui::Text("delta time: %f", delT);
            if(delT > 1.0 / 60.0) {
                ImGui::SameLine();
                ImGui::Text("BAD FRAMES");
            }
        }
        ImGui::EndTabBar();
    }
    ImGui::End();

    physics_manager.update(delT);
    particle_manager.update(delT);
    //delete when out of frame
    for(auto& r : rigidbodies) {
        if(!AABBvAABB(aabb_outer, r->aabb())) {
            if(selection.selected.contains(r))
                selection.selected.erase(r);
            physics_manager.unbind(r);
            delFromCircs(r);
            delFromPolys(r);
            rigidbodies.erase(r);
            break;
        }
    }
}
void Sim::draw() {
    //drawing
    window.clear(PastelColor::bg4);
    drawFill(window, *selection.trigger, PastelColor::Purple);

    for(auto& r : rigidbodies) {
        DrawRigidbody(r, selection.selected, window);
    }
    particle_manager.draw(window);
    for(auto p : polygon_creation_vec) {
        float r = 5.f;
        sf::CircleShape c(r);
        c.setPosition(p - vec2f(r, r));
        c.setFillColor(Color::Magenta);
        window.draw(c);
    }

}
void Sim::Run() {
    sf::Clock DeltaClock;

    while (window.isOpen())
    {
        sf::Event event;
        auto delT = DeltaClock.restart();
        FPS.push_back(1.f / delT.asSeconds());
        total_sim_time += delT.asSeconds();

        if(max_sim_time < total_sim_time)
            return;

        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);
            onEvent(event, delT.asSeconds());
        }
        ImGui::SFML::Update(window, delT);

        update(delT.asSeconds());
        draw();
        ImGui::SFML::Render(window);
        window.display();
    }

}
Sim::~Sim() {
    std::cout << "total simulation time: " << total_sim_time << "\n";
    auto avg = 0.f;
    FPS.erase(FPS.begin(), FPS.begin() + 5);
    for(auto f : FPS) {
        avg += f / (float)FPS.size();
    }
    std::cout << "average fps during simulation: " << avg << "\n";
    ImGui::SFML::Shutdown(window);
}
Sim::Sim(float w, float h, Circle c, size_t c_count, Polygon p, size_t p_count, float sim_time)
      : m_width(w), m_height(h), window(sf::VideoMode(w, h), "collisions"),
        max_sim_time(sim_time), physics_manager(AABB(vec2f(0, 0), vec2f(w, h)) )
{
    for(size_t i = 0; i < c_count; i++) {
        c.pos -= vec2f(1.f, 1.f);
        circs.emplace_back(std::make_unique<RigidCircle>(RigidCircle(c)));
        rigidbodies.emplace(circs.back().get());
        physics_manager.bind(circs.back().get());
    }
    for(size_t i = 0; i < p_count; i++) {
        p.setPos(p.getPos() - vec2f(1.f, 1.f));
        polys.emplace_back(std::make_unique<RigidPolygon>(RigidPolygon(p)));
        rigidbodies.emplace(polys.back().get());
        physics_manager.bind(polys.back().get());
    }

    ImGui::SFML::Init(window);

    setupImGuiFont();
    setup();
}

}
