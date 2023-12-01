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
    std::vector<ConvexPolygon> shape;
public:
    Tag tag;
    Tag mask;
    bool isTrigger = false;

    Collider* parent_collider = this;
    float time_immobile = 0.f;
    bool isSleeping = false;

    ConvexPolygon getPolygonShape(Transform& trans) const {
        auto t = shape.front();
        t.setPos(trans.getPos());
        t.setRot(trans.getRot());
        t.setScale(trans.getScale());
        return t;
    }

    virtual AABB getAABB(Transform& trans) { 
        auto t = shape.front();
        t.setPos(trans.getPos());
        t.setRot(trans.getRot());
        t.setScale(trans.getScale());
        return AABB::CreateFromPolygon(t);
    }
    float calcInertia(float mass) {
        return calculateInertia(shape.front().getModelVertecies(), mass); 
    }
    float getInertia(float mass) {
        if(m_inertia_dev_mass == -1.f) {
            m_inertia_dev_mass = calcInertia(mass) / mass;
        }
        return m_inertia_dev_mass * mass;
    }

    Collider() = delete;
    Collider(ConvexPolygon poly) { 
        shape = {poly};
    }
    virtual ~Collider() {
    }
};

}
