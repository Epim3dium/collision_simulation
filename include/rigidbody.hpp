#pragma once
#include "col_utils.hpp"
#include "imgui.h"
#include "types.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace EPI_NAMESPACE {

enum class eRigidShape {
    Polygon,
    Circle
};
struct Rigidbody;
struct Collider {
private:
    inline static size_t getNextLayer() {
        static size_t id = 0;
        return ++id;
    }
public:
#define MIN_DORMANT_TIME 5.f
    bool isDormant = false;
    bool isDormantCapable() const {return dormant_time > MIN_DORMANT_TIME;}
    float dormant_time = 0;
    vec2f last_pos = vec2f(0, 0);
    float pressure = 0.f;

    size_t layer;
    bool lockRotation = false;
    Rigidbody* now_colliding = nullptr;
    Collider() : layer(getNextLayer()) {}
};
struct Material {
    float restitution = 0.0f;
    float sfriction = 0.4f;
    float dfriction = 0.4f;
    float air_drag = 0.0001f;
};

//struct made only to prevent ids from copying
struct RigidbodyIdentificators {
private:
    inline static size_t getNextID()  {
        static size_t s_id = 1;
        return s_id++;
    }
    size_t m_id;
public:
    inline size_t getID() const {
        return m_id;
    }
    RigidbodyIdentificators(const RigidbodyIdentificators&) : m_id(getNextID()) {}
    RigidbodyIdentificators() : m_id(getNextID()) {}
};

class Rigidbody : public RigidbodyIdentificators {
public:
    bool isStatic = false;

    vec2f velocity;
    vec2f acceleration;

    float angular_velocity = 0.f;
    float mass = 1.f;
    Material material;
    Collider collider;

    //get inertia of custom shape
    virtual float inertia() const = 0;
    //update pos with valocity
    //virtual void updateMovement(float delT = 1.f) = 0;
    virtual vec2f getPos() const = 0;
    virtual void setPos(vec2f) = 0;
    virtual float getRot() const = 0;
    virtual void setRot(float) = 0;

    //get type identificator
    virtual eRigidShape getType() const = 0;
    
    //get axis aligned bounding box of custom shape
    virtual AABB aabb() const = 0;
    //resolve overlap function for rigidbodies with diffrent types

    inline void addForce(vec2f force) {
        velocity += force / mass;
    }
};

//calculating inertia of polygon shape
float getInertia(vec2f pos, const std::vector<vec2f>& model, float mass);

class RigidPolygon  : public Rigidbody, public Polygon {
private:
    float m_inertia;
    inline void onChange() {
        m_inertia = getInertia(vec2f(0, 0), getModelVertecies(), mass);
    }
public:
    void addVelocity(vec2f dir, vec2f cp);
    void addForce(vec2f dir, vec2f cp);
    inline float inertia() const override { 
        return getInertia(vec2f(0, 0), getModelVertecies(), mass); 
    }
    virtual vec2f getPos() const override {
        return Polygon::getPos();
    }
    void setPos(vec2f p) override {
        Polygon::setPos(p);
    }
    float getRot() const override {
        return Polygon::getRot();
    }
    void setRot(float r) override {
        Polygon::setRot(r);
    }

    inline eRigidShape getType() const  override  {
        return eRigidShape::Polygon;
    }
    AABB aabb() const override {
        return this->getAABB();
    }

    RigidPolygon(const Polygon& poly) : Polygon(poly) { 
        onChange(); 
    }
    RigidPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Polygon(pos_, rot_, model_) { 
        onChange(); 
    }
};
struct RigidCircle : public Rigidbody, public Circle {
    AABB m_aabb;
    public:
    float rot;
    inline float inertia() const override {
        return 0.25f * mass * radius * radius;
    }
    virtual vec2f getPos() const override {
        return pos;
    }
    void setPos(vec2f p) override {
        pos = p;
        m_aabb = {pos - vec2f(radius, radius), pos + vec2f(radius, radius) };
    }
    float getRot() const override {
        return rot;
    }
    void setRot(float r) override {
        rot = r;
    }

    inline eRigidShape getType() const  override  {
        return eRigidShape::Circle;
    }
    AABB aabb() const override {
        return m_aabb;
    }
    //bool detectPossibleOverlap(Rigidbody* other) override;
    RigidCircle(const RigidCircle&) = default;
    RigidCircle(const Circle& c) : Circle(c) {}
    RigidCircle(vec2f pos, float radius) : Circle(pos, radius) {}
};
}
