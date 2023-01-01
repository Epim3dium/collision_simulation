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
    size_t a_vert;
    RigidPolygon* b;
    size_t b_vert;
    float dist = 0.f;
    void update(float delT) override;
    RestraintPoint(float distance, RigidPolygon* r1, size_t r1_vert, RigidPolygon* r2, size_t r2_vert)
        : a(r1), a_vert(r1_vert), b(r2), b_vert(r2_vert), dist(distance) { }
};

}
