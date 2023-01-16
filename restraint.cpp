#include "restraint.hpp"
#include "col_utils.hpp"
namespace EPI_NAMESPACE {
void RestraintVertex::update(float delT) {
    auto diff = a->getVertecies()[a_vert] - b->getVertecies()[b_vert];
    auto l = len(diff);
    if(l > dist) {
        auto off = (l - dist) / 2.f * 0.9f;
        auto n = diff / l;
        if(!b->isStatic)
            b->addForce(off * n * (1.f + a->isStatic) * b->mass, b->getVertecies()[b_vert]);
        if(!a->isStatic)
            a->addForce(-off * n * (1.f + b->isStatic) * a->mass, a->getVertecies()[a_vert]);
    }
}
void RestraintPoint::update(float delT) {
    auto ap = rotateVec(model_point_a, a->getRot()) + a->getPos();
    auto bp = rotateVec(model_point_b, b->getRot()) + b->getPos();
    auto diff = ap - bp;
    auto l = len(diff);
    if(l > dist) {
        auto off = (l - dist) / 2.f * 0.9f;
        auto n = diff / l;
        if(!b->isStatic)
            b->addForce(off * n * (1.f + a->isStatic) * b->mass, bp);
        if(!a->isStatic)
            a->addForce(-off * n * (1.f + b->isStatic) * a->mass, ap);
    }
}
void RestraintDistance::update(float delT) {
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
}
