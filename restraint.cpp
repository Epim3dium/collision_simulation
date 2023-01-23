#include "restraint.hpp"
#include "col_utils.hpp"
namespace EPI_NAMESPACE {
void RestraintPoint::update(float delT) {
    auto& ref_a = a->collider;
    auto& ref_b = b->collider;

    auto ap = rotateVec(model_point_a, ref_a.getRot()) + ref_a.getPos();
    auto bp = rotateVec(model_point_b, ref_b.getRot()) + ref_b.getPos();
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
    auto diff = a->getCollider().getPos() - b->getCollider().getPos();
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
