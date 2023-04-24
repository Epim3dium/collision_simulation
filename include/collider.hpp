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
    Circle
};
struct Rigidbody;
//calculating inertia of polygon shape
float getInertia(vec2f pos, const std::vector<vec2f>& model, float mass);

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
struct Collider : public GameObject, public Signal::Observer {
protected:
    Transform* _transform;
public:
    //if nothing is in collision_mask then it can collide with anything
    Tag mask;
    #define COLLIDER_TYPE (typeid(Collider).hash_code())
    Property getPropertyList() const override {
        return {COLLIDER_TYPE, "collider"};
    }
    virtual eCollisionShape getType() const = 0;


    virtual AABB getAABB() const = 0;
    virtual float calcInertia(float mass) const = 0;

    Collider(Transform* trans) {
        trans->addObserver(this);
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
    void onNotify(const GameObject& obj, Signal::Event event) override {
        if(obj.getPropertyList().type_hashed == TRANSFORM_TYPE) {
            if(event == Signal::Transform::EventRotChange) {
            } else if(event == Signal::Transform::EventPosChange) {
                m_shape.pos = cast<Transform>(obj).getPos();
            } else if(event == Signal::Transform::EventScaleChange) {
                assert(false);
            } 
        }
    }
    inline float calcInertia(float mass) const override {
        return 0.25f * mass * m_shape.radius * m_shape.radius;
    }

    Circle getShape() const {
        return m_shape;
    }

    AABB getAABB() const override {
        return AABB::CreateMinMax(m_shape.pos - vec2f(m_shape.radius, m_shape.radius), m_shape.pos+ vec2f(m_shape.radius, m_shape.radius) );
    }
    CircleCollider(Transform* trans, const Circle& c) : Collider(trans), m_shape(c) { }
    CircleCollider(Transform* trans, vec2f pos, float radius) : Collider(trans), m_shape(pos, radius) { }
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
    void onNotify(const GameObject& obj, Signal::Event event) override {
        if(obj.getPropertyList().type_hashed == TRANSFORM_TYPE) {
            auto& trans = cast<Transform>(obj);
            if(event == Signal::Transform::EventRotChange) {
                m_shape.setRot(trans.getRot());
            } else if(event == Signal::Transform::EventPosChange) {
                m_shape.setPos(trans.getPos());
            } else if(event == Signal::Transform::EventScaleChange) {
                m_shape.setScale(trans.getScale());
            } 
        }
    }

    inline float calcInertia(float mass) const override { 
        return getInertia(vec2f(0, 0), m_shape.getModelVertecies(), mass); 
    }
    Polygon getShape() const {
        return m_shape;
    }
    void setShape(const Polygon& poly) {
        m_shape = poly;
        m_shape.setPos(_transform->getPos());
        m_shape.setRot(_transform->getRot());
        m_shape.setScale(_transform->getScale());
    }

    AABB getAABB() const override {
        return AABB::CreateFromPolygon(m_shape);
    }

    PolygonCollider(Transform* trans, const Polygon& poly) : m_shape(poly), Collider(trans) { 
        m_shape.setPos(trans->getPos());
        m_shape.setRot(trans->getRot());
        m_shape.setScale(trans->getScale());
    }
    PolygonCollider(Transform* trans, vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Collider(trans), m_shape(pos_, rot_, model_) { 
        m_shape.setPos(trans->getPos());
        m_shape.setRot(trans->getRot());
        m_shape.setScale(trans->getScale());
    }
};

}
