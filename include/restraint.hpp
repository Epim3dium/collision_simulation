#pragma once
#include "rigidbody.hpp"

namespace EPI_NAMESPACE {

struct RestraintInterface {
    virtual void update(float delT) = 0;
};

struct RestraintDistance : public RestraintInterface {
    Rigidbody* a;
    Rigidbody* b;
    float dist = 0.f;
    void update(float delT);
    RestraintDistance(float distance, Rigidbody* r1, Rigidbody* r2) : a(r1), b(r2), dist(distance) { }
};
struct RestraintPoint : public RestraintInterface {
    RigidPolygon* a;
    vec2f model_point_a;
    RigidPolygon* b;
    vec2f model_point_b;
    float dist = 0.f;
    void update(float delT) override;
    RestraintPoint(float distance, RigidPolygon* r1, vec2f model_pa, RigidPolygon* r2, vec2f model_pb)
        : a(r1), model_point_a(model_pa), b(r2), model_point_b(model_pb), dist(distance) { }
};

}
