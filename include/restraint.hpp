#pragma once
#include "rigidbody.hpp"

namespace EPI_NAMESPACE {

struct RestraintInterface {
    virtual void update(float delT) = 0;
};

struct DistanceRestraint : public RestraintInterface {
    Rigidbody* a;
    Rigidbody* b;
    float dist = 0.f;
    void update(float delT) override {
        auto diff = a->getPos() - b->getPos();
        auto l = len(diff);
        if(l > dist) {
            auto off = (l - dist) / 2.f;
            auto n = diff / l;
            if(!b->isStatic)
                b->velocity += off * n * (1.f + a->isStatic);
            if(!a->isStatic)
                a->velocity -= off * n * (1.f + b->isStatic);
        }
    }
    DistanceRestraint(float distance, Rigidbody* r1, Rigidbody* r2) : a(r1), b(r2), dist(distance) { }
};

}
