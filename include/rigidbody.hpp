#pragma once
#include "col_utils.hpp"
#include "imgui.h"
#include "types.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <sys/_types/_size_t.h>
#include <vector>
#include <set>

namespace EPI_NAMESPACE {

enum class eCollisionShape {
    Polygon,
    Circle
};
struct Rigidbody;
//calculating inertia of polygon shape
float getInertia(vec2f pos, const std::vector<vec2f>& model, float mass);

struct ColliderInterface {
    virtual eCollisionShape getType() const = 0;
    virtual AABB getAABB() const = 0;
    virtual vec2f getPos() const = 0;
    virtual void setPos(vec2f) = 0;
    virtual float getRot() const = 0;
    virtual void setRot(float) = 0;
    virtual float inertia(float mass) const = 0;
};
struct ColliderCircle : public ColliderInterface, public Circle {
    AABB m_aabb;
    public:
    float rot;
    inline float inertia(float mass) const override {
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

    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Circle;
    }
    AABB getAABB() const override {
        return m_aabb;
    }
    ColliderCircle(const Circle& c) : Circle(c) {}
    ColliderCircle(vec2f pos, float radius) : Circle(pos, radius) {}
};
struct ColliderPolygon : public ColliderInterface, public Polygon {
private:
public:
    void addVelocity(vec2f dir, vec2f cp);
    void addForce(vec2f dir, vec2f cp);
    inline float inertia(float mass) const override { 
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

    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Polygon;
    }
    AABB getAABB() const override {
        return AABB::CreateFromPolygon(*this);
    }

    ColliderPolygon(const Polygon& poly) : Polygon(poly) { 
    }
    ColliderPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Polygon(pos_, rot_, model_) { 
    }
};

struct Material {
    float restitution = 0.0f;
    float sfriction = 0.4f;
    float dfriction = 0.4f;
    float air_drag = 0.001f;
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
    std::set<size_t> collision_mask = {1};
    std::set<size_t> collision_layer = {1};

    bool isStatic = false;
    bool lockRotation = false;

    vec2f velocity;
    float angular_velocity = 0.f;

    float mass = 1.f;
    Material material;

    float pressure = 0.f;
    float time_immobile = 0.f;

    bool isDormant() const {
        return time_immobile > 5.f || isStatic;
    }

    virtual ColliderInterface& getCollider() = 0;
    const ColliderInterface& getCollider() const {
        return getCollider();
    }

    float inertia() {
        return getCollider().inertia(mass);
    }
    eCollisionShape getType() const  {
        return getCollider().getType();
    }
    inline void addForce(vec2f force) {
        velocity += force / mass;
    }
    void addVelocity(vec2f dir, vec2f cp);
    void addForce(vec2f dir, vec2f cp);
};


class RigidPolygon  : public Rigidbody {
private:
public:
    ColliderPolygon collider;
    ColliderInterface& getCollider() override {
        return collider;
    }

    RigidPolygon(const Polygon& poly) : collider(poly) { 
    }
    RigidPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : collider(pos_, rot_, model_) { 
    }
};
struct RigidCircle : public Rigidbody {
    AABB m_aabb;
    public:
    float rot;
    ColliderCircle collider;
    ColliderInterface& getCollider() override {
        return collider;
    }
    //bool detectPossibleOverlap(Rigidbody* other) override;
    RigidCircle(const RigidCircle&) = default;
    RigidCircle(const Circle& c) : collider(c) {}
    RigidCircle(vec2f pos, float radius) : collider(pos, radius) {}
};
}
