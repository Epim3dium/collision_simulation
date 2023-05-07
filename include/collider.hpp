#pragma once
#include "col_utils.hpp"
#include "game_object.hpp"
#include "game_object_utils.hpp"
#include "imgui.h"
#include "transform.hpp"
#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <sys/_types/_size_t.h>
#include <vector>
#include <set>

namespace epi {
//diffrent types of colliders
enum class eCollisionShape {
    Polygon,
    Circle,
    Ray
};
struct Rigidbody;
//calculating inertia of polygon shape
float calculateInertia(vec2f pos, const std::vector<vec2f>& model, float mass);

/*
* \brief Interface Class for creating collider classes , an extension of GAMEOBJECT
*(has prop list and notifies of death)
*( one of 3 key components to collsion simulation)
* virtual functions: 
*   AABB getAABB()
*   eCollisionShape getType()
*   float calcInertia()
*Collider(Transform* trans)
*/
struct Collider : public GameObject {
    float m_inertia_dev_mass = -1.f;
protected:
    virtual float calcInertia(float mass) const = 0;
public:
    //if nothing is in collision_mask then it can collide with anything
    Tag mask;
    #define COLLIDER_TYPE (typeid(Collider).hash_code())
    Property getPropertyList() const override {
        return {COLLIDER_TYPE, "collider"};
    }
    virtual eCollisionShape getType() const = 0;


    virtual AABB getAABB(Transform& trans) const = 0;
    float getInertia(float mass) {
        if(m_inertia_dev_mass == -1.f) {
            m_inertia_dev_mass = calcInertia(mass) / mass;
        }
        return m_inertia_dev_mass * mass;
    }

    Collider() {
    }
    virtual ~Collider() {
        notify(*this, Signal::EventDestroyed);
    }
};

/*
* \brief derived from Collider, a collider in the shape of a circle
*   CircleCollider(Transform* trans, const Circle& c) : Collider(trans), m_shape(c) { }
*   CircleCollider(Transform* trans, vec2f pos, float radius) : Collider(trans), m_shape(pos, radius) { }
*/
struct CircleCollider : public Collider {
    AABB m_aabb;
    Circle m_shape;
public:
    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Circle;
    }
    inline float calcInertia(float mass) const override {
        return 0.25f * mass * m_shape.radius * m_shape.radius;
    }

    Circle getShape(Transform& trans) const {
        auto t = m_shape;
        t.pos = trans.getPos();
        return std::move(t);
    }

    AABB getAABB(Transform& trans) const override {
        auto t = m_aabb;
        t.setCenter(trans.getPos());
        return t;
    }
    CircleCollider(const Circle& c) : Collider(), m_shape(c) { 
        m_aabb = AABB::CreateMinMax(m_shape.pos - vec2f(m_shape.radius, m_shape.radius), m_shape.pos+ vec2f(m_shape.radius, m_shape.radius) );
    }
    CircleCollider(vec2f pos, float radius) : Collider(), m_shape(pos, radius) { 
        m_aabb = AABB::CreateMinMax(m_shape.pos - vec2f(m_shape.radius, m_shape.radius), m_shape.pos+ vec2f(m_shape.radius, m_shape.radius) );
    }
};
/*
* \brief derived from Collider, a collider in the shape of a polygon 
*   PolygonCollider(Transform* trans, const Polygon& poly) : m_shape(poly), Collider(trans) { 
*   PolygonCollider(Transform* trans, vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Collider(trans), m_shape(pos_, rot_, model_) { 
*/
struct PolygonCollider : public Collider {
private:
    Polygon m_shape;
    vec2f m_scale = {1.f, 1.f};
public:
    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Polygon;
    }

    inline float calcInertia(float mass) const override { 
        return calculateInertia(vec2f(0, 0), m_shape.getModelVertecies(), mass); 
    }
    Polygon getShape(Transform& trans) const {
        auto t = m_shape;
        t.setPos(trans.getPos());
        t.setRot(trans.getRot());
        t.setScale(trans.getScale());
        return std::move(t);
    }
    void setShape(const Polygon& poly) {
        m_shape = poly;
    }

    AABB getAABB(Transform& trans) const override {
        auto t = m_shape;
        t.setPos(trans.getPos());
        t.setRot(trans.getRot());
        t.setScale(trans.getScale());
        return AABB::CreateFromPolygon(t);
    }

    PolygonCollider(const Polygon& poly) : m_shape(poly), Collider() { 
    }
    PolygonCollider(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Collider(), m_shape(pos_, rot_, model_) { 
    }
};
/*
* \brief derived from Collider, a collider in the shape of a polygon 
*   PolygonCollider(Transform* trans, const Polygon& poly) : m_shape(poly), Collider(trans) { 
*   PolygonCollider(Transform* trans, vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Collider(trans), m_shape(pos_, rot_, model_) { 
*/
struct RayCollider : public Collider {
private:
    Ray m_shape;
    vec2f m_scale = {1.f, 1.f};
public:
    inline eCollisionShape getType() const  override  {
        return eCollisionShape::Ray;
    }

    inline float calcInertia(float mass) const override { 
        return 1.f / 12.f * mass * qlen(m_shape.dir); 
    }
    Ray getShape(Transform& trans) const {
        auto t = m_shape;
        t.dir = rotateVec(t.dir, trans.getRot());
        t.pos = trans.getPos();
        t.pos -= t.dir / 2.f;
        return std::move(t);
    }
    void setShape(const Ray& r) {
        m_shape = r;
    }

    AABB getAABB(Transform& trans) const override {
        auto t = getShape(trans);
        vec2f min, max;
        min.x = std::min(t.pos.x, t.pos.x + t.dir.x);
        min.y = std::min(t.pos.y, t.pos.y + t.dir.y);
        max.x = std::max(t.pos.x, t.pos.x + t.dir.x);
        max.y = std::max(t.pos.y, t.pos.y + t.dir.y);
        return AABB::CreateMinMax(min, max);
    }
    RayCollider(Ray ray) : Collider(), m_shape(ray) { 
    }
};

}
