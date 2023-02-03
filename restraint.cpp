#include "restraint.hpp"
#include "col_utils.hpp"
namespace EPI_NAMESPACE {
void RestraintPoint::update(float delT) {
    auto& ref_a = a->collider;
    auto& ref_b = b->collider;

    auto ap = rotateVec(model_point_a, ref_a.getRot()) + ref_a.getPos() + a->velocity * delT;
    auto bp = rotateVec(model_point_b, ref_b.getRot()) + ref_b.getPos() + b->velocity * delT;
    auto diff = ap - bp;
    auto l = len(diff);
    if(l > dist) {
        auto off = (l - dist);
        auto n = diff / l;
        if(!b->isStatic)
            b->addVelocity(off * n * (1.f + a->isStatic), bp);
        if(!a->isStatic)
            a->addVelocity(-off * n * (1.f + b->isStatic), ap);
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
