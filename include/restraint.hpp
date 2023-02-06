#pragma once
#include "rigidbody.hpp"
#include <vector>

namespace EPI_NAMESPACE {

struct RestraintInterface {
    virtual void update(float delT) = 0;
    virtual std::vector<Rigidbody*> getRestrainedObjects() const = 0;
    virtual ~RestraintInterface() {}
};

struct RestraintDistance : public RestraintInterface {
    Rigidbody* a;
    Rigidbody* b;
    float dist = 0.f;
    void update(float delT) override;
    std::vector<Rigidbody*> getRestrainedObjects() const override {
        return {a, b};
    }

    RestraintDistance(float distance, Rigidbody* r1, Rigidbody* r2) : a(r1), b(r2), dist(distance) { }
};
struct RestraintPoint : public RestraintInterface {
    RigidPolygon* a;
    vec2f model_point_a;
    RigidPolygon* b;
    vec2f model_point_b;
    float dist = 0.f;
    void update(float delT) override;
    std::vector<Rigidbody*> getRestrainedObjects() const override {
        return {(Rigidbody*)a, (Rigidbody*)b};
    }
    RestraintPoint(float distance, RigidPolygon* r1, vec2f model_pa, RigidPolygon* r2, vec2f model_pb)
        : a(r1), model_point_a(model_pa), b(r2), model_point_b(model_pb), dist(distance) { }
};

}
