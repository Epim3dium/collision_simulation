#include <memory>
#include <string>
#include <sys/_types/_size_t.h>
#include <sys/signal.h>
#include <vector>
#include <numeric>

#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/System/Time.hpp"
#include "SFML/Window/Event.hpp"
#include "SFML/Window/Keyboard.hpp"
#include "SFML/Window/Mouse.hpp"
#include "debug.hpp"
#include "imgui-SFML.h"

#include "io_manager.hpp"
#include "types.hpp"
#include "col_utils.hpp"
#include "collider.hpp"
#include "imgui.h"
#include "restraint.hpp"
#include "rigidbody.hpp"
#include "scene.hpp"
#include "transform.hpp"
#include "RNG.h"

using namespace epi;

static void DrawRigid(RigidManifold man, sf::RenderTarget& rw, Color color = PastelColor::bg1) {
    auto& col = *man.collider;
    switch(col.type) {
        case eCollisionShape::Circle: {
            auto c = col.getCircleShape(*man.transform);
            sf::CircleShape cs(c.radius);
            cs.setPosition(man.transform->getPos() - vec2f(c.radius, c.radius));
            cs.setFillColor(color);
            cs.setOutlineColor(Color::Black);
            DEBUG_CALL(cs.setOutlineColor(Color::Red));
            cs.setOutlineThickness(1.f);
            rw.draw(cs);

            DEBUG_CALL(
            sf::Vertex verts[2];
            verts[0].position = c.pos;
            verts[1].position = c.pos + rotateVec(vec2f(c.radius, 0.f), man.transform->getRot());
            verts[0].color = Color::Blue;
            verts[1].color = Color::Blue;
            rw.draw(verts, 2, sf::Lines);
            )
        }break;
        case eCollisionShape::Polygon: {
            auto p = col.getPolygonShape(*man.transform);
            drawFill(rw, p, color);
            drawOutline(rw, p, sf::Color::Black);
            DEBUG_CALL(drawOutline(rw, p, sf::Color::Red));
        } break;
        case epi::eCollisionShape::Ray: {
            Ray t = col.getRayShape(*man.transform);
            sf::Vertex verts[2] ;
            verts[0].position = t.pos;
            verts[1].position = t.pos + t.dir;
            verts[0].color = sf::Color::White;
            verts[1].color = sf::Color::White;
            rw.draw(verts, 2, sf::Lines);
        } break;
    }
}
#define CONSOLAS_PATH "assets/Consolas.ttf"
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
struct CollisionLogger : public Signal::Observer<ColliderEvent>  {
    bool isLogging = false;
    void onNotify(ColliderEvent event) {
        auto cn = event.info.cn;
        if(isLogging){
            Log(LogLevel::INFO, 0.25f) << &event.me << " collided with " << &event.other <<
                ", with a collision normal of: " << cn.x << " " << cn.y;
        }
    }
};
class DemoObject {
public:
    std::unique_ptr<Transform> transform;
    std::unique_ptr<Collider> collider;
    std::unique_ptr<Rigidbody> rigidbody;
    std::unique_ptr<Material> material;
    RigidManifold getManifold() const {
        return {transform.get(), collider.get(), rigidbody.get(), material.get()};
    }
    DemoObject(Polygon poly) {
        transform = std::unique_ptr<Transform>(new Transform());
        transform->setPos(poly.getPos());
        transform->setRot(poly.getRot());
        collider = std::unique_ptr<Collider>(new Collider(poly));
        rigidbody = std::unique_ptr<Rigidbody>(new Rigidbody());
        material = std::unique_ptr<Material>(new Material());
    }
    DemoObject(Circle circ) {
        transform = std::unique_ptr<Transform>(new Transform());
        transform->setPos(circ.pos);
        collider = std::unique_ptr<Collider>(new Collider(circ));
        rigidbody = std::unique_ptr<Rigidbody>(new Rigidbody());
        material = std::unique_ptr<Material>(new Material());
    }
    DemoObject(Ray ray) {
        transform = std::unique_ptr<Transform>(new Transform());
        transform->setPos(ray.pos + ray.dir / 2.f);
        collider = std::unique_ptr<Collider>(new Collider(ray));
        rigidbody = std::unique_ptr<Rigidbody>(new Rigidbody());
        rigidbody->isStatic = true;
        material = std::unique_ptr<Material>(new Material());
    }
    ~DemoObject() {
    }
};
class Demo : public DefaultScene {
protected:
    std::vector<std::unique_ptr<DemoObject>> demo_objects;
    struct {
        RNG _rng;
        float default_radius = 50.f;
        float radius_dev = 0.1f;
        float gravity = 1000.f;
        std::vector<vec2f> poly_creation;
        struct {
            int min = 4;
            int max = 5;
        } poly_sides_count;
        struct {
            DemoObject* object = nullptr;
            bool isHolding = false;
            Restraint* res;
            Transform* mouse_trans = new Transform();
            vec2f pinch_point;
            CollisionLogger logger;
        } selection;
    }opts;
    DemoObject* findHovered() {
        DemoObject* result;
        auto mouse_pos = (vec2f)io_manager.getMousePos();
        for(auto& r : demo_objects) {
            switch(r->collider->type) {
                case eCollisionShape::Circle: {
                    Circle shape = r->collider->getCircleShape(*r->transform);
                    if(isOverlappingPointCircle(mouse_pos, shape)) {
                        return r.get();
                    }
                } break;
                case eCollisionShape::Polygon: {
                    Polygon polygon = r->collider->getPolygonShape(*r->transform);
                    if(isOverlappingPointPoly(mouse_pos, polygon)) {
                        return r.get();
                    }
                }break;
                case eCollisionShape::Ray: {
                    Ray t = r->collider->getRayShape(*r->transform);
                    auto closest = findClosestPointOnRay(t.pos, t.dir, mouse_pos);
                    if(len(closest - mouse_pos) < 10.f)
                        return r.get();
                }break;
            }
        }
        return nullptr;
    }

    void onSetup() override {
        setupImGuiFont();
        physics_manager.steps = 5;
        physics_manager.bounciness_select = PhysicsManager::eSelectMode::Max;
        physics_manager.friction_select = PhysicsManager::eSelectMode::Max;

        auto aabb_outer = sim_window;
        auto aabb_inner = sim_window;
        static const float padding = 80.f;
        aabb_inner.setSize(aabb_inner.size() - vec2f(padding * 2.f, padding * 2.f));
        aabb_outer.setSize(aabb_outer.size() - vec2f(padding, padding));
        {
            AABB trigger = aabb_inner;
            trigger.setCenter({10000.f, 10000.f});
            //selection.trigger = std::make_unique<SelectingTrigger>(SelectingTrigger(Polygon::CreateFromAABB(trigger)));
            //_physics_manager->add(selection.trigger);
        }
        //RigidBody model_poly = Polygon(vec2f(), 0.f, mini_model);
#define ADD_SIDE(ax, ay, bx, by)\
        {\
            std::vector<vec2f> model;\
            model.push_back(vec2f(aabb_inner.ax, aabb_inner.ay));\
            model.push_back(vec2f(aabb_inner.bx, aabb_inner.by));\
            model.push_back(vec2f(aabb_outer.bx, aabb_outer.by));\
            model.push_back(vec2f(aabb_outer.ax, aabb_outer.ay));\
            auto t = Polygon::CreateFromPoints(model);\
            demo_objects.push_back(std::unique_ptr<DemoObject>(new DemoObject(t)));\
            demo_objects.back().get()->rigidbody->isStatic = true;\
            demo_objects.back().get()->collider->tag.add("ground");\
            physics_manager.add(demo_objects.back().get()->getManifold());\
        }

        ADD_SIDE(min.x, min.y, min.x, max.y);
        ADD_SIDE(max.x, min.y, min.x, min.y);
        ADD_SIDE(max.x, max.y, min.x, max.y);
        ADD_SIDE(max.x, max.y, max.x, min.y);

    }
    void onEvent(const sf::Event& event) {
        switch(event.type) 
        {
            case sf::Event::MouseButtonPressed: {
                if(event.mouseButton.button == sf::Mouse::Left) {
                    auto hovered = findHovered();
                    if(hovered) {
                        //opts.selection.isHolding = true;
                        opts.selection.pinch_point = rotateVec((vec2f)io_manager.getMousePos() - hovered->transform->getPos(), -hovered->transform->getRot());
                        opts.selection.res = new RestraintPointTrans( hovered->getManifold(), opts.selection.pinch_point, opts.selection.mouse_trans, vec2f());
                        physics_manager.add(opts.selection.res);
                        opts.selection.object = hovered;
                        opts.selection.isHolding = true;
                    }else {
                        opts.poly_creation.push_back((vec2f)io_manager.getMousePos());
                    }
                } else if(event.mouseButton.button == sf::Mouse::Right) {
                    auto hovered = findHovered();
                    if(hovered && opts.selection.object) {
                        auto dist = hovered->transform->getPos() - opts.selection.object->transform->getPos();
                        auto a = opts.selection.object;
                        auto ap = opts.selection.pinch_point;
                        auto b = hovered;
                        auto bp = rotateVec((vec2f)io_manager.getMousePos() - hovered->transform->getPos(), -hovered->transform->getRot());
                        auto res = new RestraintRigidRigid(b->getManifold(), bp, a->getManifold(), ap);
                        physics_manager.add(res);
                    }
                }
            }break;
            case sf::Event::MouseButtonReleased: {
                if(opts.selection.isHolding && !sf::Keyboard::isKeyPressed(sf::Keyboard::X)) {
                    physics_manager.remove(opts.selection.res);
                    delete opts.selection.res;
                }else if(sf::Keyboard::isKeyPressed(sf::Keyboard::X)){
                    opts.selection.mouse_trans = new Transform();
                    opts.selection.res = nullptr;
                }
                opts.selection.isHolding = false;
            }break;
            case sf::Event::KeyPressed: {
                float r = opts.default_radius * opts._rng.Random(1.f, 1.f + opts.radius_dev);
                switch(event.key.code)
                {
                    case sf::Keyboard::C: {
                        Circle t((vec2f)io_manager.getMousePos(), r);
                        demo_objects.push_back(std::unique_ptr<DemoObject>(new DemoObject(t)));
                        physics_manager.add(demo_objects.back().get()->getManifold());
                    }break;
                    case sf::Keyboard::V: {
                        auto side_count = opts._rng.Random(opts.poly_sides_count.min, opts.poly_sides_count.max);
                        Polygon t = Polygon::CreateRegular((vec2f)io_manager.getMousePos(), M_PI/side_count, side_count, r * sqrt(2.f));
                        auto ptr = new DemoObject(t);
                        demo_objects.push_back(std::unique_ptr<DemoObject>(ptr));
                        physics_manager.add(demo_objects.back().get()->getManifold());
                    }break;
                    case sf::Keyboard::Enter: {
                        if(opts.poly_creation.size() < 2) {
                            break;
                        }else if(opts.poly_creation.size() == 2) {
                            Ray t = Ray::CreatePoints(opts.poly_creation.front(), opts.poly_creation.back());
                            demo_objects.push_back(std::unique_ptr<DemoObject>(new DemoObject(t)));
                        } else {
                            Polygon t = Polygon::CreateFromPoints(opts.poly_creation);
                            demo_objects.push_back(std::unique_ptr<DemoObject>(new DemoObject(t)));
                        }
                        physics_manager.add(demo_objects.back().get()->getManifold());
                        opts.selection.object = demo_objects.back().get();
                    }break;
                    case sf::Keyboard::BackSpace: {
                        DemoObject* objptr = opts.selection.object;
                        do {
                            auto itr = std::find_if(demo_objects.begin(), demo_objects.end(), 
                                [&](const std::unique_ptr<DemoObject>& obj) {
                                    return obj.get() == objptr;
                                });
                            if(itr != demo_objects.end()) {
                                physics_manager.remove(itr->get()->getManifold());
                                demo_objects.erase(itr);
                            }
                            objptr = findHovered();
                        }while(objptr != nullptr);
                        opts.selection.object = nullptr;
                        opts.selection.isHolding = false;
                    }break;
                    case sf::Keyboard::Space: {
                        if(opts.selection.object)
                            opts.selection.object->rigidbody->isStatic = !opts.selection.object->rigidbody->isStatic;
                    }break;
                    case sf::Keyboard::R: {
                        if(opts.selection.isHolding)
                            delete opts.selection.res;
                        opts.selection.object = nullptr;
                        opts.poly_creation.clear();
                    }break;
                    default:
                    break;
                }break;
            }break;
            default:
            break;
        }
    }
    void onNotify(IOManagerEvent msg) override {
        if(msg.event.type == sf::Event::Closed) {
            bail("window closed");
        }
        //demo
        if (!ImGui::IsAnyItemHovered()) {
            onEvent(msg.event);
        }
    }

    void onUpdate(float delT) override {
        opts.selection.mouse_trans->setPos((vec2f)io_manager.getMousePos());
        if(opts.selection.isHolding && opts.selection.object && opts.selection.object->rigidbody->isStatic) {
            opts.selection.object->transform->setPos((vec2f)io_manager.getMousePos() - rotateVec(opts.selection.pinch_point, opts.selection.object->transform->getRot()));
        }
        for(auto& r : demo_objects) {
            if(!r.get()->rigidbody->isStatic)
                r.get()->rigidbody->force += vec2f(0, opts.gravity) * r->rigidbody->mass;
        }
        if(opts.selection.object)
            opts.selection.object->collider->addObserver(&opts.selection.logger);

        physics_manager.update(delT);

        if(opts.selection.object)
            opts.selection.object->collider->removeObserver(&opts.selection.logger);

        for(auto it = demo_objects.begin(); it != demo_objects.end(); it++) {
            if(!isOverlappingPointAABB(it->get()->transform->getPos(), sim_window)) {
                demo_objects.erase(it);
                physics_manager.remove(it->get()->getManifold());
                break;
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
                    static int framerate_max = 0xffffff;
                    ImGui::SliderInt("change max fps" , &framerate_max, 1, 1000);
                    this->io_manager.getWindow().setFramerateLimit(framerate_max);

                    static int tsteps = 5;
                    ImGui::SliderInt("change step count" , &tsteps, 1, 50);
                    physics_manager.steps = tsteps;
                    ImGui::SliderFloat("change gravity" , &opts.gravity, -3000.f, 3000.f, "%.1f");
                    ImGui::SliderFloat("radius" , &opts.default_radius, 5.f, 100.f);
                    ImGui::SliderFloat("radius_deviation" , &opts.radius_dev, 0.f, 1.f);
                    ImGui::Text("poly_sides range:");
                    ImGui::SliderInt("poly_sides_min", &opts.poly_sides_count.min, 3, 10);
                    ImGui::SliderInt("poly_sides_max", &opts.poly_sides_count.max, 4, 11);
                    if(opts.poly_sides_count.min >= opts.poly_sides_count.max) {
                        opts.poly_sides_count.max = opts.poly_sides_count.min + 1;
                    }
                    const char* select_modes[] = { "Min", "Max", "Avg" };
                    {
                        static int cur_choice_friction = 2;
                        ImGui::ListBox("choose mode friction", &cur_choice_friction, select_modes, 3);
                        physics_manager.friction_select = (PhysicsManager::eSelectMode)cur_choice_friction;
                    }
                    {
                        static int cur_choice_bounce = 2;
                        ImGui::ListBox("choose mode bounce", &cur_choice_bounce, select_modes, 3);
                        physics_manager.bounciness_select = (PhysicsManager::eSelectMode)cur_choice_bounce;
                    }ImGui::EndTabItem();
                } 
            }
            {
                static bool open_object = true;
                if(ImGui::BeginTabItem("object settings", &open_object)) {
                    auto obj = opts.selection.object;
        
                    if(!obj) {
                        if(ImGui::Button("toggle Logging")) {
                            opts.selection.logger.isLogging = !opts.selection.logger.isLogging;
                        }
                        ImGui::Text("NO SELECTION");
                    }else {
                        ImGui::SliderFloat("mass: ", &obj->rigidbody->mass, 1.f, 20.f);
                        ImGui::SliderFloat("static_fric: ", &obj->material->sfriction, 0.f, 1.f);
                        ImGui::SliderFloat("dynamic_fric: ", &obj->material->dfriction, 0.f, 1.f);
                        ImGui::SliderFloat("bounciness: ", &obj->material->restitution, 0.f, 1.f);
                        ImGui::SliderFloat("drag: ", &obj->material->air_drag, 0.f, 1.f);
                        if(ImGui::Button("lockRotation"))
                            obj->rigidbody->lockRotation = !obj->rigidbody->lockRotation;
                        if(ImGui::Button("isStatic"))
                            obj->rigidbody->isStatic = !obj->rigidbody->isStatic;

                        ImGui::Text("===TAGS===");
                        auto tags = obj->collider->tag.getList();
                        for(auto t : tags) {
                            ImGui::Text("->%s", t.c_str());
                            ImGui::SameLine();
                            std::string st = t;
                            st = "remove " + st;
                            if(ImGui::Button(st.c_str())) {
                                obj->collider->tag.remove(t);
                            }
                        }
                        static std::string input_tag;
                        if(ImGui::InputText("add_field_tag", (char*)input_tag.c_str(), input_tag.capacity() + 1, ImGuiInputTextFlags_EnterReturnsTrue)) {
                            obj->collider->tag.add(input_tag.c_str());
                            input_tag = "";
                        }

                        ImGui::Text("===MASKS===");
                        auto masks = obj->collider->mask.getList();
                        for(auto m : masks) {
                            ImGui::Text("->%s", m.c_str());
                            ImGui::SameLine();
                            std::string sm = m;
                            sm = "remove " + sm;
                            if(ImGui::Button(sm.c_str())) {
                                obj->collider->mask.remove(m);
                            }
                        }
                        static std::string input_mask;
                        if(ImGui::InputText("add_field_mask", (char*)input_mask.c_str(), input_tag.capacity() + 1, ImGuiInputTextFlags_EnterReturnsTrue)) {
                            obj->collider->mask.add(input_mask.c_str());
                            input_mask = "";
                        }
                    }
                    ImGui::EndTabItem();
                }
            }
            ImGui::Text("total bodies: %d", int(demo_objects.size()));
            ImGui::Text("delta time: %f", delT);
            ImGui::Text("FPS: %f", 1.f / delT);
            if(delT > 1.0 / 60.0) {
                ImGui::SameLine();
                ImGui::Text("BAD FRAMES");
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }
    void onRender(sf::RenderTarget& target) override {
        target.clear();
        //demo
        for(auto& r : demo_objects) {
            auto& rb = *r->rigidbody;
            Color color = PastelColor::bg1;
            if(!rb.isStatic) {
                color = PastelColor::Aqua;
            }
            if(r->collider->isSleeping) {
                color = PastelColor::Yellow;
            }
            if(r.get() == opts.selection.object) {
                color = PastelColor::Red;
            }
            DrawRigid(r.get()->getManifold(), target, color);
        }
        for(auto& v : opts.poly_creation) {
            const float r = 5.f;
            sf::CircleShape c(r);
            c.setPosition(v - vec2f(r, r));
            c.setFillColor(PastelColor::Red);
            target.draw(c);
        }
        //DEBUG_CALL(debugDraw(target, _physics_manager->getQuadTree(), Color::Magenta));
    }
public:
    Demo(vec2i s = {1500, 1500}) : DefaultScene(s) {}
};

int main() {
    DEBUG_CALL(Log(LogLevel::DEBUG) << "debug build running");
    Demo demo;
    run(demo);
    return 0;
}
