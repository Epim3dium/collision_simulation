#pragma once
#include "col_utils.hpp"
#include "rigidbody.hpp"
#include "types.hpp"

namespace EPI_NAMESPACE {
struct TriggerInterface {
    virtual eCollisionShape getType() const = 0;
    virtual void onActivation(Rigidbody* rb, vec2f cn) = 0;
    virtual AABB aabb() const = 0;

    struct DetectionResult {
        bool detected = false;
        vec2f contact_normal;
    };

    virtual DetectionResult detectTrigger(Rigidbody* rb1) = 0;
};
struct TriggerCircleInterface : public TriggerInterface, public Circle {
    eCollisionShape getType() const override {
        return eCollisionShape::Circle;
    }
    DetectionResult detectTrigger(Rigidbody* rb1) override {
        bool result;
        vec2f cn;
        switch(rb1->getType()) {
            case eCollisionShape::Polygon:
                result = detect(*this, *(RigidPolygon*)rb1, &cn, nullptr, nullptr);
            break;
            case eCollisionShape::Circle:
                result = detect(*(RigidCircle*)rb1, *this, &cn, nullptr, nullptr);
            break;
        }
        return {result, cn};
    }
    AABB aabb() const override {
        return AABBfromCircle(*this);
    }
    TriggerCircleInterface(const Circle& shape) : Circle(shape) {}
};
struct TriggerPolygonInterface : public TriggerInterface, public Polygon {
    eCollisionShape getType() const override {
        return eCollisionShape::Polygon;
    } 
    DetectionResult detectTrigger(Rigidbody* rb1) override {
        bool result;
        vec2f cn;
        switch(rb1->getType()) {
            case eCollisionShape::Polygon:
                result = detect(*(RigidPolygon*)rb1, *this, &cn, nullptr);
            break;
            case eCollisionShape::Circle:
                result = detect(*(RigidCircle*)rb1, *this, &cn, nullptr, nullptr);
            break;
        }
        return {result, cn};
    }
    AABB aabb() const override {
        return AABBfromPolygon(*this);
    }
    TriggerPolygonInterface(const Polygon& shape) : Polygon(shape) {}
};

}
