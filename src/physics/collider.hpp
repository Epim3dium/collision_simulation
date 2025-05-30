#pragma once
#include "col_utils.hpp"
#include "imgui.h"
#include "transform.hpp"
#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
#include <set>

namespace epi {
class Rigidbody;
class Collider;

struct CollisionInfo {
    bool detected;
    vec2f cn;
    std::vector<vec2f> cps;
    float overlap;
};
//diffrent types of colliders
enum class eCollisionShape {
    Polygon,
    Circle,
    Ray
};
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
struct ColliderEvent {
    Collider& me;
    Collider& other;
    CollisionInfo info;
};
class Collider : public Signal::Subject<ColliderEvent> {
    float m_inertia_dev_mass = -1.f;
    union {
        struct {
            AABB aabb;
            Circle shape;
        }_circle;
        struct {
            Polygon shape;
            vec2f scale;
        }_polygon;
        struct {
            Ray shape;
            vec2f scale;
        }_ray;
    };
public:
    Tag tag;
    Tag mask;
    bool isTrigger = false;
    const eCollisionShape type;

    Collider* parent_collider = this;
    float time_immobile = 0.f;
    bool isSleeping = false;

    Circle getCircleShape(Transform& trans) const {
        assert(type == eCollisionShape::Circle);
        auto t = _circle.shape;
        t.pos = trans.getPos();
        return t;
    }
    Polygon getPolygonShape(Transform& trans) const {
        assert(type == eCollisionShape::Polygon);
        auto t = _polygon.shape;
        t.setPos(trans.getPos());
        t.setRot(trans.getRot());
        t.setScale(trans.getScale());
        return t;
    }
    Ray getRayShape(Transform& trans) const {
        auto t = _ray.shape;
        t.dir = rotateVec(t.dir, trans.getRot());
        t.pos = trans.getPos();
        t.pos -= t.dir / 2.f;
        return t;
    }

    virtual AABB getAABB(Transform& trans) { 
        switch(type) {
            case eCollisionShape::Circle: {
                auto t = _circle.aabb;
                t.setCenter(trans.getPos());
                return t;
            }
            case eCollisionShape::Polygon: {
                auto t = _polygon.shape;
                t.setPos(trans.getPos());
                t.setRot(trans.getRot());
                t.setScale(trans.getScale());
                return AABB::CreateFromPolygon(t);
            }
            case eCollisionShape::Ray: {
                auto t = getRayShape(trans);
                vec2f min, max;
                min.x = std::min(t.pos.x, t.pos.x + t.dir.x);
                min.y = std::min(t.pos.y, t.pos.y + t.dir.y);
                max.x = std::max(t.pos.x, t.pos.x + t.dir.x);
                max.y = std::max(t.pos.y, t.pos.y + t.dir.y);
                return AABB::CreateMinMax(min, max);
            }

        }
    }
    float calcInertia(float mass) {
        switch(type) {
            case eCollisionShape::Circle:
                return 0.25f * mass * _circle.shape.radius * _circle.shape.radius;
            case eCollisionShape::Polygon:
                return calculateInertia(vec2f(0, 0), _polygon.shape.getModelVertecies(), mass); 
            case eCollisionShape::Ray:
                return 0.08333f * mass * qlen(_ray.shape.dir); 
        }
    }
    float getInertia(float mass) {
        if(m_inertia_dev_mass == -1.f) {
            m_inertia_dev_mass = calcInertia(mass) / mass;
        }
        return m_inertia_dev_mass * mass;
    }

    Collider() = delete;
    Collider(Ray ray) : type(eCollisionShape::Ray) { 
        _ray.shape = ray;
    }
    Collider(Polygon poly) : type(eCollisionShape::Polygon) { 
        _polygon.shape = poly;
    }
    Collider(Circle c) : type(eCollisionShape::Circle) {
        _circle.shape = c;
        _circle.aabb = AABB::CreateMinMax(_circle.shape.pos - vec2f(_circle.shape.radius, _circle.shape.radius), _circle.shape.pos+ vec2f(_circle.shape.radius, _circle.shape.radius) );
    }
    virtual ~Collider() {
    }
};

}
