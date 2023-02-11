#include "restraint.hpp"
#include "col_utils.hpp"
#include <cmath>
namespace EPI_NAMESPACE {
void RestraintPoint::update(float delT) {
    auto& ref_a = a->getCollider();
    auto& ref_b = b->getCollider();

    auto ap = rotateVec(model_point_a, ref_a.getRot()) + ref_a.getPos() + a->velocity * delT;
    auto bp = rotateVec(model_point_b, ref_b.getRot()) + ref_b.getPos() + b->velocity * delT;
    auto diff = ap - bp;
    auto l = len(diff);
    if(abs(l) > dist * 0.1f) {
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
void RestraintRotation::update(float delT) {
    auto diff = a->getCollider().getPos() - (b->getCollider().getPos());
    auto l = len(diff);
    float cur_angle = atan2(diff.y, diff.x);

    auto a_desired_angle =  cur_angle - a_angle + M_PI;
    auto b_desired_angle =  cur_angle - b_angle + M_PI;
    a_desired_angle= fmod( a_desired_angle, M_PI * 2.f);
    b_desired_angle= fmod( b_desired_angle, M_PI * 2.f);

    if(!b->isStatic) {
        std::cerr << "b\n";
        auto end = rotateVec(vec2f(1.0f, 0.f), a->getCollider().getRot() + a_angle) * dist + a->getCollider().getPos();
        b->velocity += (end - b->getCollider().getPos()) * delT * 1000.f;
        b->getCollider().setRot(b_desired_angle);
    }
    if(!a->isStatic) {
        auto end = rotateVec(vec2f(1.0f, 0.f), b->getCollider().getRot() + b_angle + M_PI) * dist + b->getCollider().getPos();
        a->velocity += (end - a->getCollider().getPos()) * delT * 1000.f;
        a->getCollider().setRot(a_desired_angle);
    }

}
}
