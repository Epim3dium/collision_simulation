#include "sim.hpp"
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Vertex.hpp"
#include "SFML/Window/Keyboard.hpp"
#include "SFML/Window/Mouse.hpp"
#include "col_utils.hpp"
#include "imgui.h"
#include "restraint.hpp"
#include "rigidbody.hpp"
#include "types.hpp"
#include "crumbler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <sys/_types/_size_t.h>
#include <thread>
#include <vector>

#define DEBUG_DRAW true 

namespace EPI_NAMESPACE {
void SelectingTrigger::onActivation(Rigidbody* rb, vec2f cn) {
    if(selected.contains(rb))
        return;
    selected.emplace(rb);
}
static bool PointVCollider(vec2f p, ColliderInterface& rb) {
    switch(rb.getType()) {
        case eCollisionShape::Circle:
            return isOverlappingPointCircle(p, (ColliderCircle&)rb);
        break;
        case eCollisionShape::Polygon:
            return isOverlappingPointPoly(p, (ColliderPolygon&)rb);
        break;
    }
}
Color blend(Color c1, Color c2, float t) {
    t = std::clamp(t, 0.f, 1.f);
    c1.r *= (1.f - t);
    c1.g *= (1.f - t);
    c1.b *= (1.f - t);

    c2.r *= t;
    c2.g *= t;
    c2.b *= t;

    c1.r += c2.r;
    c1.g += c2.g;
    c1.b += c2.b;
    return c1;
}
static void DrawRigidbody(Rigidbody* rb, const std::set<Rigidbody*>& selection, sf::RenderWindow& rw) {
    Color color = PastelColor::bg1;
    if(!rb->isStatic) {
        color = PastelColor::Aqua;
    }
    if(rb->pressure != 0.f)
        color = blend(color, Color::Cyan, rb->pressure / 100.f);
    if(rb->isDormant() && !rb->isStatic)
        color = PastelColor::Yellow;
    if(selection.contains(rb))
        color = PastelColor::Red;
    auto& col = rb->getCollider();
    switch(col.getType()) {
        case eCollisionShape::Circle: {
            ColliderCircle c((ColliderCircle&)col);
            sf::CircleShape cs(c.radius);
            cs.setPosition(c.pos - vec2f(c.radius, c.radius));
            cs.setFillColor(color);
            cs.setOutlineColor(Color::Black);
            cs.setOutlineThickness(1.f);
            rw.draw(cs);
            cs.setOutlineThickness(0.f);
            float r = c.radius / 2.f;
            cs.setRadius(r);
            cs.setPosition(c.pos + vec2f(cos(c.rot), sin(c.rot)) * c.radius / 2.f - vec2f(r, r));
            color = blend(color, Color::Black, 0.2f);
            cs.setFillColor(color);
            rw.draw(cs);
        }break;
        case eCollisionShape::Polygon:
            drawFill(rw, (ColliderPolygon&)col, color);
            drawOutline(rw, (ColliderPolygon&)col, sf::Color::Black);
        break;
    }
#if DEBUG_DRAW
    AABB t = rb->getCollider().getAABB();
    drawOutline(rw, Polygon::CreateFromAABB(t), PastelColor::Purple);
#endif

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
void Sim::crumbleSquarely(Rigidbody* poly) {
    auto size = std::max(poly->getCollider().getAABB().size().x, poly->getCollider().getAABB().size().y) * 0.4f;
    Crumbler crumbler(size, size * 0.3f);

    auto devisions = crumbler.crumble(poly->getCollider());
    float total_area = 0.f;
    for(auto& d : devisions) {
        total_area += area(d.getModelVertecies());
    }
    vec2f pos = poly->getCollider().getPos();
    vec2f vel = poly->velocity;
    float angvel = poly->angular_velocity;
    float mass = poly->mass;

    removeRigidbody(poly);
    for(auto& p : devisions) {
        RigidPolygon t(p);

        vec2f rad = t.getCollider().getPos() - pos;
        vec2f radperp(-rad.y, rad.x);
        vec2f pang_vel_lin = radperp * angvel;

        float cur_area = area(p.getModelVertecies());
        float cur_mass = cur_area / total_area * mass;

        t.velocity = vel + pang_vel_lin;
        t.mass = cur_mass;

        if(cur_area > 200.f) {
            createRigidbody(t);
        }else if(cur_area > 50.f) {
            RigidCircle c(t.getCollider().getPos(), sqrt(cur_area / 3.141f));
            c.velocity = vel + pang_vel_lin;
            c.mass = cur_mass;
            createRigidbody(c);
        }
    }
}
void Sim::setup() {
    physics_manager.steps = 10;
    physics_manager.bounciness_select = PhysicsManager::eSelectMode::Max;
    physics_manager.friction_select = PhysicsManager::eSelectMode::Max;

    aabb_inner.setSize(aabb_inner.size() - vec2f(padding * 2.f, padding * 2.f));
    {
        AABB trigger = aabb_inner;
        trigger.setCenter({10000.f, 10000.f});
        selection.trigger = std::make_unique<SelectingTrigger>(SelectingTrigger(Polygon::CreateFromAABB(trigger)));
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
        createRigidbody(t);\
    }

    ADD_SIDE(min.x, min.y, min.x, max.y);
    ADD_SIDE(max.x, min.y, min.x, min.y);
    ADD_SIDE(max.x, max.y, min.x, max.y);
    ADD_SIDE(max.x, max.y, max.x, min.y);

}
void Sim::onEvent(const sf::Event &event, float delT) {
    if (event.type == sf::Event::Closed)
        window.close();
    if (ImGui::IsAnyItemHovered())
        return;
    else if(event.type == sf::Event::MouseButtonPressed) {
        if(event.mouseButton.button == sf::Mouse::Button::Left) {
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
                Rigidbody* sel = nullptr;
                auto mpos = (vec2f)sf::Mouse::getPosition(window);
                for(auto r : rigidbodies) {
                    if(PointVCollider(mpos, r->getCollider())) {
                        sel = r;
                        break;
                    }
                }
                if(selection.last_restrain_sel && sel) {
                    auto selP = rotateVec(mpos - sel->getCollider().getPos(), -sel->getCollider().getRot());
                    auto last_selP = selection.last_restrain_sel_off;
                    auto* res = new RestraintPoint(len(selection.last_mouse_pos - mpos)
                        , (RigidPolygon*)sel, selP, (RigidPolygon*)selection.last_restrain_sel, last_selP);
                    restraints.push_back(res);
                    physics_manager.bind(res);
                    selection.last_restrain_sel = nullptr;
                } else if(sel){
                    selection.last_mouse_pos = mpos;
                    selection.last_restrain_sel = sel; 
                    selection.last_restrain_sel_off = rotateVec(mpos - sel->getCollider().getPos(), -sel->getCollider().getRot());

                }
            }else {
                selection.last_mouse_pos = (vec2f)sf::Mouse::getPosition(window);
                for(auto r : rigidbodies) {
                    if(PointVCollider(selection.last_mouse_pos, r->getCollider())) {
                        if(!selection.selected.contains(r)) {
                            selection.selected = {r};
                        }
                        selection.isHolding = true;
                        break;
                    }
                }
                if(selection.isHolding) {
                    for(auto s : selection.selected) {
                        selection.offsets[s] = s->getCollider().getPos() - selection.last_mouse_pos;
                        s->lockRotation = true;
                    }
                }else {
                    selection.isMaking = true;
                }
            }
        }
    }
    else if(event.type == sf::Event::MouseButtonReleased) {
        if(event.mouseButton.button == sf::Mouse::Button::Left) {
            if(selection.isHolding) {
                for(auto s : selection.selected) {
                    s->lockRotation = false;
                }
            }
            else if(selection.making_time < 0.25f && selection.isMaking) {
                polygon_creation_vec.push_back(selection.last_mouse_pos);
            }
            selection.isMaking = false;
            selection.making_time = 0;
            selection.isHolding = false;

            selection.trigger->setShape({});
        }
        else if(event.mouseButton.button == sf::Mouse::Button::Right) {
        }
    }
    else if(event.type == sf::Event::KeyPressed) {
        if(event.key.control && event.key.code == sf::Keyboard::A) {
            selection.selected = rigidbodies;
        } else if(event.key.code == sf::Keyboard::Enter && polygon_creation_vec.size() > 2) {
            RigidPolygon t(Polygon::CreateFromPoints(polygon_creation_vec));
            createRigidbody(t);
            polygon_creation_vec.clear();
        }else if(event.key.code == sf::Keyboard::X) {
            for(auto rb : selection.selected) {
                crumbleSquarely(rb);
            }
            selection.selected.clear();
            polygon_creation_vec.clear();
        }else if(event.key.code == sf::Keyboard::R) {
            selection.selected.clear();
            polygon_creation_vec.clear();
            selection.last_restrain_sel = nullptr;
        }else if (event.key.code == sf::Keyboard::C) {
            RigidCircle t((vec2f)sf::Mouse::getPosition(window), default_dynamic_radius);
            createRigidbody(t);
        }else if (event.key.code == sf::Keyboard::V) {
            vec2f mpos = (vec2f)sf::Mouse::getPosition(window);
            RigidPolygon t = Polygon::CreateRegular(mpos, 3.141 / 4.f, 4U, default_dynamic_radius * sqrt(2.f));
            createRigidbody(t);
        }
    }
}
void Sim::update(float delT) {
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
        vec2f mpos = (vec2f)sf::Mouse::getPosition(window);
        particle_manager.emit(3000U * delT, Particle::InitList(
            Particle::PosInit(mpos),
            Particle::LifetimeInit(0.15f, 0.2f),
            Particle::ColorVecInit({PastelColor::Red}),
            Particle::ShapeInit(5.f),
            Particle::VelMagInit(1200.f, 1200.f),
            Particle::VelAngleInit(-3.141f, 3.141f),
            Particle::ColorFuncInit(
                [](Color clr, float time)->Color {
                    clr = blend(clr, Color::White, 0.1f);
                    return blend(PastelColor::Yellow, clr, time);
                })
            )
        );
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
        for(auto s : selection.selected) {
            s->angular_velocity += 10.f * delT;
        }
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
        for(auto s : selection.selected) {
            s->angular_velocity -= 10.f * delT;
        }
    }
    if(selection.isHolding) {
        for(auto s : selection.selected) {
            auto d = (vec2f)sf::Mouse::getPosition(window) + selection.offsets[s] - s->getCollider().getPos();
            if(!s->isStatic) {
                s->velocity = d / sqrt(delT);
            } else {
                s->getCollider().setPos((vec2f)sf::Mouse::getPosition(window) + selection.offsets[s]);
            }
            //s->getCollider().setPos((vec2f)sf::Mouse::getPosition(window) + selection.offsets[s]);
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
        if(isOverlappingPointPoly(mpos, p.collider)) {
            found = &p;
            break;
        }
    }
    for(auto& c : circs) {
        if(isOverlappingPointCircle(mpos, c.collider)) {
            found = &c;
            break;
        }
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::BackSpace)) {
        for(auto s : selection.selected) {
            removeRigidbody(s);
        }
        selection.selected.clear();
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
        for(auto s : selection.selected)
            s->isStatic = true;
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
        for(auto s : selection.selected)
            s->isStatic = false;
    }

    ImGui::Begin("Demo window");
    {
        static int tab_open = 0;
        ImGui::BeginTabBar("Settings");
        {
            static bool open_global = true;
            if(ImGui::BeginTabItem("global settings", &open_global))
            {
                static int tsteps = 5;
                ImGui::SliderInt("change step count" , &tsteps, 1, 50);
                physics_manager.steps = tsteps;
                ImGui::SliderFloat("change gravity" , &gravity, -3000.f, 3000.f, "%.1f");
                ImGui::SliderFloat("radius" , &default_dynamic_radius, 1.f, 500.f);
                const char* select_modes[] = { "Min", "Max", "Avg" };
                {
                    static int cur_choice_friction = 0;
                    ImGui::ListBox("choose mode friction", &cur_choice_friction, select_modes, 3);
                    physics_manager.friction_select = (PhysicsManager::eSelectMode)cur_choice_friction;
                }
                {
                    static int cur_choice_bounce = 0;
                    ImGui::ListBox("choose mode bounce", &cur_choice_bounce, select_modes, 3);
                    physics_manager.bounciness_select = (PhysicsManager::eSelectMode)cur_choice_bounce;
                }ImGui::EndTabItem();
            } 
            static bool open_object = true;
            if(ImGui::BeginTabItem("object settings", &open_object))
            {
                Rigidbody* hovered_last = *selection.selected.begin();
                if(selection.selected.size() != 0 && hovered_last) {
                    if(ImGui::Button("isStatic")) {
                        hovered_last->isStatic = !hovered_last->isStatic;
                    }
                    if(ImGui::Button("LockRotation")) {
                        hovered_last->lockRotation = !hovered_last->lockRotation;
                    }
                    ImGui::Text("properties of selected object");
                    ImGui::SliderFloat("change mass" , &hovered_last->mass, 1.0f, 100.f);
                    ImGui::SliderFloat("change air_drag" , &hovered_last->material.air_drag, 0.0f, 0.01f, "%.5f");
                    ImGui::SliderFloat("change static fric" , &hovered_last->material.sfriction, 0.0f, 1.f);
                    ImGui::SliderFloat("change dynamic fric" , &hovered_last->material.dfriction, 0.0f, 1.f);
                    ImGui::SliderFloat("change restitution" , &hovered_last->material.restitution, 0.0f, 1.f);
                }else {
                    ImGui::Text("NONE SELECTED");
                }
                ImGui::EndTabItem();
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

    //delete when out of frame
    for(auto& r : rigidbodies) {
        r->velocity += vec2f(0.f, gravity) * delT;
        if(!isOverlappingAABBAABB(aabb_outer, r->getCollider().getAABB())) {
            if(selection.selected.contains(r))
                selection.selected.erase(r);
            removeRigidbody(r);
            break;
        }
    }
    physics_manager.update(delT, &particle_manager);
    particle_manager.update(delT);
}
void Sim::draw() {
    //drawing
    window.clear(PastelColor::bg4);
    drawFill(window, *selection.trigger, PastelColor::Purple);

    for(auto& r : rigidbodies) {
        DrawRigidbody(r, selection.selected, window);
    }
#if DEBUG_DRAW
    for(auto& r : restraints) {
        auto cons = r->getRestrainedObjects();
        auto prev = cons.back();
        for(auto cur : cons) {
            sf::Vertex vert[2];
            vert[0].color = Color::Cyan;
            vert[0].position = prev->getCollider().getPos();
            vert[1].color = Color::Cyan;
            vert[1].position = cur->getCollider().getPos();
            window.draw(vert, 2, sf::Lines);
        }
    }
#endif
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
            ImGui::SFML::ProcessEvent(window, event);
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
Sim::Sim(float w, float h)
      : m_width(w), m_height(h), window(sf::VideoMode(w, h), "collisions"), physics_manager({{-w * 3.f, -h * 3.f}, {w * 3.f, h * 3.f}})
{

    ImGui::SFML::Init(window);

    setupImGuiFont();
    setup();
}

}
