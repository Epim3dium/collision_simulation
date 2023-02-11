#pragma once
#include "col_utils.hpp"
#include "rigidbody.hpp"
#include "types.hpp"

namespace EPI_NAMESPACE {
struct TriggerInterface {
    virtual ColliderInterface& getCollider() = 0;
    virtual void onActivation(Rigidbody* rb, vec2f cn) = 0;
};
struct TriggerCircleInterface : public TriggerInterface, public ColliderCircle {
    ColliderInterface& getCollider() override {
        return *this;
    }
    TriggerCircleInterface(const Circle& shape) : ColliderCircle(shape) {}
};
struct TriggerPolygonInterface : public TriggerInterface, public ColliderPolygon {
    ColliderInterface& getCollider() override {
        return *this;
    } 
    TriggerPolygonInterface(const Polygon& shape) : ColliderPolygon(shape) {}
};

}
