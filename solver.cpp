#include "solver.hpp"
#include "rigidbody.hpp"
#include "trigger.hpp"

#include <algorithm>
#include <exception>
#include <random>
#include <cmath>
#include <numeric>
#include <vector>


namespace EPI_NAMESPACE {

CollisionManifold detectOverlap(ColliderCircle& c1, ColliderPolygon& c2) {

    auto intersection = intersectCirclePolygon(c1.getShape(), c2.getShape());
    if(intersection.detected) {
        return {true, nullptr, nullptr, c1.getPos(), c2.getPos(), intersection.contact_normal, {intersection.contact_point} , intersection.overlap};
    }
    return {false};
}
CollisionManifold detectOverlap(ColliderPolygon& c1, ColliderPolygon& c2) {
    auto intersection = intersectPolygonPolygon(c1.getShape(), c2.getShape());
    if(intersection.detected) {
        std::vector<vec2f> cps;
        cps = findContactPoints(c1.getShape(), c2.getShape());
        if(cps.size() > 2)
            std::cerr << cps.size() << "\n";
        return {true, nullptr, nullptr, c1.getPos(), c2.getPos(), intersection.contact_normal, cps , intersection.overlap};
    }
    return {false};
}
CollisionManifold detectOverlap(ColliderCircle& r1, ColliderCircle& r2) {
    auto intersection = intersectCircleCircle(r1.getShape(), r2.getShape());
    if(intersection.detected) {
        return {true, nullptr, nullptr, r1.getPos(), r2.getPos(), intersection.contact_normal, {intersection.contact_point} , intersection.overlap};
    }
    return {false};
}
void handleOverlap(Rigidbody& a, Rigidbody& b, const CollisionManifold& man) {
    auto& r1 = *man.r1;
    auto& r2 = *man.r2;
    if(!man.detected)
        return;
    if(r2.isDormant()) {
        r1.getCollider().setPos(r1.getCollider().getPos() + man.cn * man.overlap);
    } else if(r1.isDormant()) {
        r2.getCollider().setPos(r2.getCollider().getPos() - man.cn * man.overlap);
    } else {
        r1.getCollider().setPos(r1.getCollider().getPos() + man.cn * man.overlap / 2.f);
        r2.getCollider().setPos(r2.getCollider().getPos() - man.cn * man.overlap / 2.f);
    }
}
void DefaultSolver::processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
       vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, const std::vector<vec2f>& cps)
{
    float mass1 = rb1.isStatic ? INFINITY : rb1.mass;
    float mass2 = rb2.isStatic ? INFINITY : rb2.mass;
    float inv_inertia1 = rb1.isStatic || rb1.lockRotation ? INFINITY : rb1.inertia();
    float inv_inertia2 = rb2.isStatic || rb2.lockRotation ? INFINITY : rb2.inertia();

    if(inv_inertia1 != 0.f)
        inv_inertia1 = 1.f / inv_inertia1;
    if(inv_inertia2 != 0.f)
        inv_inertia2 = 1.f / inv_inertia2;


    vec2f impulse (0, 0);
    vec2f additional_rb1vel(0, 0); float additional_rb1ang_vel = 0.f;
    vec2f additional_rb2vel(0, 0); float additional_rb2ang_vel = 0.f;

    for(auto& cp : cps) {
        vec2f rad1 = cp - pos1;
        vec2f rad2 = cp - pos2;

        vec2f rad1perp(-rad1.y, rad1.x);
        vec2f rad2perp(-rad2.y, rad2.x);

        vec2f p1ang_vel_lin = rb1.lockRotation ? vec2f(0, 0) : rad1perp * rb1.angular_velocity;
        vec2f p2ang_vel_lin = rb2.lockRotation ? vec2f(0, 0) : rad2perp * rb2.angular_velocity;

        vec2f vel_sum1 = rb1.isStatic ? vec2f(0, 0) : rb1.velocity + p1ang_vel_lin;
        vec2f vel_sum2 = rb2.isStatic ? vec2f(0, 0) : rb2.velocity + p2ang_vel_lin;

        //calculate relative velocity
        vec2f rel_vel = vel_sum2 - vel_sum1;

        float j = getReactImpulse(rad1perp, inv_inertia1, mass1, rad2perp, inv_inertia2, mass2, bounce, rel_vel, cn);
        vec2f fj = getFricImpulse(inv_inertia1, mass1, rad1perp, inv_inertia2, mass2, rad2perp, sfric, dfric, j, rel_vel, cn);
        impulse += cn * j - fj;

        if(!rb1.isStatic) {
            additional_rb1vel -= impulse / rb1.mass;
            if(!rb1.lockRotation)
                additional_rb1ang_vel += cross(impulse, rad1) * inv_inertia1;
        }
        if(!rb2.isStatic) {
            additional_rb2vel += impulse / rb2.mass;
            if(!rb2.lockRotation)
                additional_rb2ang_vel -= cross(impulse, rad2) * inv_inertia2;
        }
    }
    float cps_count = cps.size();
    rb1.velocity         += additional_rb1vel / cps_count;
    rb1.angular_velocity += additional_rb1ang_vel / cps_count;
    rb2.velocity         += additional_rb2vel / cps_count;
    rb2.angular_velocity += additional_rb2ang_vel / cps_count;
}
void DefaultSolver::processReaction(const CollisionManifold& man, float bounce, float sfric, float dfric) {
    processReaction(man.r1pos, *man.r1, man.r1->material, man.r2pos, *man.r2, man.r2->material, bounce, sfric, dfric, man.cn, man.cps);
}
float DefaultSolver::getReactImpulse(const vec2f& rad1perp, float p1inv_inertia, float mass1, const vec2f& rad2perp, float p2inv_inertia, float mass2, 
        float restitution, const vec2f& rel_vel, vec2f cn) {
    float contact_vel_mag = dot(rel_vel, cn);
    if(contact_vel_mag < 0.f)
        return 0.f;
    //equation from net
    float r1perp_dotN = dot(rad1perp, cn);
    float r2perp_dotN = dot(rad2perp, cn);

    float denom = 1.f / mass1 + 1.f / mass2 +
        (r1perp_dotN * r1perp_dotN) *  p1inv_inertia +
        (r2perp_dotN * r2perp_dotN) *  p2inv_inertia;

    float j = -(1.f + restitution) * contact_vel_mag;
    j /= denom;
    return j;
}
vec2f DefaultSolver::getFricImpulse(float p1inv_inertia, float mass1, vec2f rad1perp, float p2inv_inertia, float mass2, const vec2f& rad2perp, 
        float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn) {
    vec2f tangent = rel_vel - cn * dot(rel_vel, cn);

    if(len(tangent) < 0.1f)
        return vec2f(0, 0);
    else
        tangent = norm(tangent);

    //equation from net
    float r1perp_dotT = dot(rad1perp, tangent);
    float r2perp_dotT = dot(rad2perp, tangent);

    float denom = 1.f / mass1 + 1.f / mass2 + 
        (r1perp_dotT * r1perp_dotT) *  p1inv_inertia +
        (r2perp_dotT * r2perp_dotT) *  p2inv_inertia;

    float contact_vel_mag = dot(rel_vel, tangent);
    float jt = -contact_vel_mag;
    jt /= denom;

    vec2f friction_impulse;
    if(abs(jt) <= abs(j * sfric)) {
        friction_impulse = tangent * -jt;
    } else {
        friction_impulse = tangent * -j * dfric;
    }
    return friction_impulse;
}
bool DefaultSolver::handle(const CollisionManifold& manifold, float restitution, float sfriction, float dfriction) {
    if(!manifold.detected)
        return false;
    processReaction(manifold.r1pos, *manifold.r1, manifold.r1->material, manifold.r2pos, 
                    *manifold.r2, manifold.r2->material, restitution, sfriction, dfriction, manifold.cn, manifold.cps);
    return true;
}
CollisionManifold DefaultSolver::detect(ColliderInterface* col1, ColliderInterface* col2, Rigidbody* rb1, Rigidbody* rb2) {
    CollisionManifold man;
    //ik its ugly but switch case will catch new variants if eCollisionShape will be getting more shapes
    switch(col1->getType()) {
        case eCollisionShape::Polygon:
            switch(col2->getType()) {
                case eCollisionShape::Polygon:
                    man = detectOverlap(*(ColliderPolygon*)col1, *(ColliderPolygon*)col2);
                    man.r1 = rb1;
                    man.r2 = rb2;
                break;
                case eCollisionShape::Circle:
                    man = detectOverlap(*(ColliderCircle*)col2, *(ColliderPolygon*)col1);
                    man.r2 = rb1;
                    man.r1 = rb2;
                break;
            }
        break;
        case eCollisionShape::Circle:
            switch(col2->getType()) {
                case eCollisionShape::Polygon:
                    man = detectOverlap(*(ColliderCircle*)col1, *(ColliderPolygon*)col2);
                    man.r1 = rb1;
                    man.r2 = rb2;
                break;
                case eCollisionShape::Circle:
                    man = detectOverlap(*(ColliderCircle*)col1, *(ColliderCircle*)col2);
                    man.r1 = rb1;
                    man.r2 = rb2;
                break;
            }
        break;
    }
    return man;
}
CollisionManifold DefaultSolver::solve(Rigidbody* rb1, Rigidbody* rb2, float restitution, float sfriction, float dfriction) {
    auto man = detect(&rb1->getCollider(), &rb2->getCollider(), rb1, rb2);
    handleOverlap(*rb1, *rb2, man);
    handle(man, restitution, sfriction, dfriction);
    return man;
}

}
