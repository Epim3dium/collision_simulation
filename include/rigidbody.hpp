#pragma once
#include "col_utils.hpp"
#include "imgui.h"
#include "transform.hpp"
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

struct ColliderInterface : public Transform {
    virtual eCollisionShape getType() const = 0;
    virtual AABB getAABB() const = 0;

    virtual vec2f getPos() const override = 0;
    virtual void setPos(vec2f) override = 0;

    virtual vec2f getScale() const override = 0;
    virtual void setScale(vec2f) override = 0;

    virtual float getRot() const override = 0;
    virtual void setRot(float) override = 0;
    virtual float inertia(float mass) const = 0;
};
struct ColliderCircle : public ColliderInterface {
    AABB m_aabb;
    Circle m_shape;
    float m_rotation = 0.f;
    vec2f m_scale = vec2f(1.f, 1.f);
    public:
    inline float inertia(float mass) const override {
        return 0.25f * mass * m_shape.radius * m_shape.radius;
    }

    virtual vec2f getPos() const override {
        return m_shape.pos;
    }
    void setPos(vec2f p) override {
        m_shape.pos = p;
        m_aabb = {p - vec2f(m_shape.radius, m_shape.radius), p + vec2f(m_shape.radius, m_shape.radius) };
    }
    float getRot() const override {
        return m_rotation;
    }
    void setRot(float r) override {
        m_rotation = r;
    }
    vec2f getScale() const override {
        return m_scale;
    }
    void setScale(vec2f s) override {
        m_scale = s;
    }
    Circle getShape() const {
        auto tmp = m_shape;
        tmp.radius *= std::max(m_scale.x, m_scale.y);
        return tmp;
    }

    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Circle;
    }
    AABB getAABB() const override {
        return m_aabb;
    }
    ColliderCircle(const Circle& c) : m_shape(c) { }
    ColliderCircle(vec2f pos, float radius) : m_shape(pos, radius) { }
};
struct ColliderPolygon : public ColliderInterface {
private:
    Polygon m_shape;
    vec2f m_scale = {1.f, 1.f};
public:
    void addVelocity(vec2f dir, vec2f cp);
    void addForce(vec2f dir, vec2f cp);

    inline float inertia(float mass) const override { 
        return getInertia(vec2f(0, 0), m_shape.getModelVertecies(), mass); 
    }
    virtual vec2f getPos() const override {
        return m_shape.getPos();
    }
    void setPos(vec2f p) override {
        m_shape.setPos(p);
    }
    float getRot() const override {
        return m_shape.getRot();
    }
    void setRot(float r) override {
        m_shape.setRot(r);
    }
    vec2f getScale() const override {
        return m_scale;
    }
    void setScale(vec2f s) override {
        m_scale = s;
    }
    Polygon getShape() const {
        auto tmp = m_shape;
        tmp.scale(m_scale);
        return tmp;
    }

    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Polygon;
    }
    AABB getAABB() const override {
        return AABB::CreateFromPolygon(m_shape);
    }

    ColliderPolygon(const Polygon& poly) : m_shape(poly) { 
    }
    ColliderPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : m_shape(pos_, rot_, model_) { 
    }
};

struct Material {
    float restitution = 0.0f;
    float sfriction = 0.4f;
    float dfriction = 0.4f;
    float air_drag = 0.001f;
};

//struct made only to prevent ids from copying
class Rigidbody : public ParentedTransform {
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
    Rigidbody(Transform* parent) : ParentedTransform(parent) {}
};


class RigidPolygon  : public Rigidbody {
private:
public:
    ColliderPolygon collider;
    ColliderInterface& getCollider() override {
        return collider;
    }

    RigidPolygon(const Polygon& poly) : collider(poly), Rigidbody(&collider) { }
    RigidPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : collider(pos_, rot_, model_), Rigidbody(&collider) { }
};
struct RigidCircle : public Rigidbody {
    AABB m_aabb;
    public:
    float rot;
    ColliderCircle collider;
    ColliderInterface& getCollider() override {
        return collider;
    }
    RigidCircle(const RigidCircle&) = default;
    RigidCircle(const Circle& c) : collider(c), Rigidbody(&collider) {}
    RigidCircle(vec2f pos, float radius) : collider(pos, radius), Rigidbody(&collider) {}
};
}
