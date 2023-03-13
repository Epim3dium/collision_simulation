#include "restraint.hpp"
#include "col_utils.hpp"
#include <cmath>
namespace EPI_NAMESPACE {
//Correction impulse = - ( object mass / timestep ) * overlap distance along axis * coefficient - velocity along axis * coefficient
void RestraintPoint::update(float delT) {
    auto& ref_a = a->getCollider();
    auto& ref_b = b->getCollider();

    vec2f ap = ref_a.getPos() + rotateVec(model_point_a, ref_a.getRot());
    vec2f bp = ref_b.getPos() + rotateVec(model_point_b, ref_b.getRot());
    auto diff = ap - bp;

    auto l = len(diff);
    auto off = (l - dist) / 2.f;
    float mag = off * off * off * 1.f / delT;
    auto n = diff / l;

    vec2f avg_vel = (a->velocity + b->velocity) / 2.f;

    if(!b->isStatic) {
        float b_corr_impulse = -mag - dot(-n, b->velocity - avg_vel) * damping_coef;
        b->addForce(-n * b_corr_impulse * delT, bp);
    }
    if(!a->isStatic){
        float a_corr_impulse = -mag - dot(n, a->velocity - avg_vel) * damping_coef;
        a->addForce(n * a_corr_impulse * delT, ap);
    }
}
void RestraintDistance::update(float delT) {
    auto diff = a->getCollider().getPos() - b->getCollider().getPos();
    auto l = len(diff);

    if(l > dist) {
        auto off = (l - dist) / 2.f;
        auto n = diff / l;
        float mag = off * off * copysign(1.f, off);

        vec2f avg_vel = (a->velocity + b->velocity) / 2.f;

        if(!b->isStatic) {
            float b_corr_impulse = -(b->mass / delT) * mag * damping_coef - dot(-n, b->velocity - avg_vel) * damping_coef;
            b->addForce(-n * b_corr_impulse * delT);
        }
        if(!a->isStatic){
            float a_corr_impulse = -(a->mass / delT) * mag * damping_coef - dot(n, a->velocity - avg_vel) * damping_coef;
            a->addForce(n * a_corr_impulse * delT);
        }

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
