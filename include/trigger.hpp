#pragma once
#include "col_utils.hpp"
#include "rigidbody.hpp"
#include "types.hpp"

namespace EPI_NAMESPACE {
struct TriggerInterface {
    virtual eRigidShape getType() const = 0;
    virtual void onActivation(Rigidbody* rb, vec2f cn) = 0;
    virtual AABB aabb() const = 0;
};
struct TriggerCircleInterface : public TriggerInterface, public Circle {
    eRigidShape getType() const override {
        return eRigidShape::Circle;
    }
    AABB aabb() const override {
        return AABBfromCircle(*this);
    }
    TriggerCircleInterface(const Circle& shape) : Circle(shape) {}
};
struct TriggerPolygonInterface : public TriggerInterface, public Polygon {
    eRigidShape getType() const override {
        return eRigidShape::Polygon;
    } 
    AABB aabb() const override {
        return Polygon::getAABB();
    }
    TriggerPolygonInterface(const Polygon& shape) : Polygon(shape) {}
};

}
