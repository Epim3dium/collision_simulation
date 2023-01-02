#pragma once
#include "col_utils.h"
#include "rigidbody.hpp"
#include "types.hpp"

namespace EPI_NAMESPACE {
struct TriggerInterface {
    virtual eRigidShape getType() const = 0;
    virtual void onActivation(Rigidbody* rb, vec2f cn) = 0;
};
struct TriggerCircleInterface : public TriggerInterface, public Circle {
    eRigidShape getType() const override {
        return eRigidShape::Circle;
    }
    TriggerCircleInterface(const Circle& shape) : Circle(shape) {}
};
struct TriggerPolygonInterface : public TriggerInterface, public Polygon {
    eRigidShape getType() const override {
        return eRigidShape::Polygon;
    } 
    TriggerPolygonInterface(const Polygon& shape) : Polygon(shape) {}
};

}
