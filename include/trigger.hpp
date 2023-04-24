#pragma once
#include "col_utils.hpp"
#include "collider.hpp"
#include "game_object.hpp"
#include "rigidbody.hpp"
#include "types.hpp"

namespace epi {
struct TriggerInterface : public GameObject {
private:
    Transform* _transform;
public:
    #define TRIGGER_TYPE (typeid(TriggerInterface).hash_code())
    Property getPropertyList() const override {
        return {TRIGGER_TYPE, "restraint"};
    }
    virtual Collider& getCollider() = 0;
    virtual void onActivation(Rigidbody* rb, vec2f cn) = 0;
    TriggerInterface(Transform* trans) : _transform(trans) {}
};
struct TriggerCircleInterface : public TriggerInterface, public CircleCollider {
    Collider& getCollider() override {
        return *this;
    }
    TriggerCircleInterface(const Circle& shape, Transform* trans) : TriggerInterface(trans), CircleCollider(trans, shape) {}
};
struct TriggerPolygonInterface : public TriggerInterface, public PolygonCollider {
    Collider& getCollider() override {
        return *this;
    } 
    TriggerPolygonInterface(const Polygon& shape, Transform* trans) : TriggerInterface(trans),PolygonCollider(trans, shape) {}
};

}
