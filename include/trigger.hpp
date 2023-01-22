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
        switch(rb1->getCollider().getType()) {
            case eCollisionShape::Polygon: {
                auto intersection = intersectCirclePolygon(*this, ((RigidPolygon*)rb1)->collider);
                return {intersection.detected, intersection.contact_normal};
            } break;
            case eCollisionShape::Circle: {
                auto intersection = intersectCircleCircle(*this, ((RigidCircle*)rb1)->collider);
                return {intersection.detected, intersection.contact_normal};
            } break;
        }
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
        switch(rb1->getCollider().getType()) {
            case eCollisionShape::Polygon: {
                auto intersection = intersectPolygonPolygon(((RigidPolygon*)rb1)->collider, *this);
                return {intersection.detected, intersection.contact_normal};
            }break;
            case eCollisionShape::Circle: {
                auto intersection = intersectCirclePolygon(((RigidCircle*)rb1)->collider, *this);
                return {intersection.detected, intersection.contact_normal};
            } break;
        }
    }
    AABB aabb() const override {
        return AABBfromPolygon(*this);
    }
    TriggerPolygonInterface(const Polygon& shape) : Polygon(shape) {}
};

}
