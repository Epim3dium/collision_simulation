#include "solver.hpp"
#include "col_utils.hpp"
#include "collider.hpp"
#include "debug.hpp"
#include "rigidbody.hpp"
#include "trigger.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <random>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>


namespace epi {

CollisionInfo detectOverlap(const Polygon& poly, const Ray& ray) {
    assert(false);
    auto intersection = intersectRayPolygon(ray.pos, ray.dir, poly); 
    if(intersection.detected) {
        return {true, intersection.contact_normal_near, {}, 10.f};
    }
    return {false};
}
CollisionInfo detectOverlap(const Circle& circle, const Ray& ray) {
    auto closest = findClosestPointOnRay(ray.pos, ray.dir, circle.pos);
    auto l = len(closest - circle.pos);
    bool detected = (l < circle.radius);
    if(detected) {
        return {true, norm(closest - circle.pos), {closest}, l - circle.radius};
    }
    return {false};
}
CollisionInfo detectOverlap(const Circle& circle, const Polygon& poly) {

    auto intersection = intersectCirclePolygon(circle, poly);
    if(intersection.detected) {
        return {true, intersection.contact_normal, {intersection.contact_point} , intersection.overlap};
    }
    return {false};
}
CollisionInfo detectOverlap(const Polygon& p1, const Polygon& p2) {
    auto intersection = intersectPolygonPolygon(p1, p2);
    if(intersection.detected) {
        std::vector<vec2f> cps;
        cps = findContactPoints(p1, p2);
        if(cps.size() ==0)
            return {false};
        return {true, intersection.contact_normal, cps , intersection.overlap};
    }
    return {false};
}
CollisionInfo detectOverlap(const Circle& c1, const Circle& c2) {
    auto intersection = intersectCircleCircle(c1, c2);
    if(intersection.detected) {
        return {true, intersection.contact_normal, {intersection.contact_point} , intersection.overlap};
    }
    return {false};
}
void handleOverlap(RigidManifold& m1, RigidManifold& m2, const CollisionInfo& man) {
    if(!man.detected)
        return;
    if(man.swapped)
        std::swap(m1, m2);
    auto& t1 = *m1.transform;
    auto& t2 = *m2.transform;
    if(m2.rigidbody->isDormant()) {
        t1.setPos(t1.getPos() + man.cn * man.overlap);
    } else if(m1.rigidbody->isDormant()) {
        t2.setPos(t2.getPos() - man.cn * man.overlap);
    } else {
        t1.setPos(t1.getPos() + man.cn * man.overlap / 2.f);
        t2.setPos(t2.getPos() - man.cn * man.overlap / 2.f);
    }
}
void DefaultSolver::processReaction(const CollisionInfo& info, const RigidManifold& m1, 
       const RigidManifold& m2,float bounce, float sfric, float dfric)
{
    //auto cp = std::reduce(info.cps.begin(), info.cps.end()) / (float)info.cps.size();

    auto& rb1 = *m1.rigidbody;
    auto& rb2 = *m2.rigidbody;

    float mass1 = rb1.isStatic ? INFINITY : rb1.mass;
    float mass2 = rb2.isStatic ? INFINITY : rb2.mass;
    float inv_inertia1 = 1.f / ((rb1.isStatic || rb1.lockRotation) ? INFINITY : m1.collider->getInertia(rb1.mass));
    float inv_inertia2 = 1.f / ((rb2.isStatic || rb2.lockRotation) ? INFINITY : m2.collider->getInertia(rb2.mass));
    vec2f vel1 = rb1.velocity;
    vec2f vel2 = rb2.velocity;
    float ang_vel1 = rb1.angular_velocity;
    float ang_vel2 = rb2.angular_velocity;


    auto cp = std::reduce(info.cps.begin(), info.cps.end()) / (float)info.cps.size();
        vec2f impulse (0, 0);

        vec2f rad1 = cp - m1.transform->getPos();
        vec2f rad2 = cp - m2.transform->getPos();

        vec2f rad1perp(-rad1.y, rad1.x);
        vec2f rad2perp(-rad2.y, rad2.x);

        vec2f p1ang_vel_lin = rb1.lockRotation ? vec2f(0, 0) : rad1perp * ang_vel1;
        vec2f p2ang_vel_lin = rb2.lockRotation ? vec2f(0, 0) : rad2perp * ang_vel2;

        vec2f vel_sum1 = rb1.isStatic ? vec2f(0, 0) : vel1 + p1ang_vel_lin;
        vec2f vel_sum2 = rb2.isStatic ? vec2f(0, 0) : vel2 + p2ang_vel_lin;

        //calculate relative velocity
        vec2f rel_vel = vel_sum2 - vel_sum1;

        float j = getReactImpulse(rad1perp, inv_inertia1, mass1, rad2perp, inv_inertia2, mass2, bounce, rel_vel, info.cn);
        vec2f fj = getFricImpulse(inv_inertia1, mass1, rad1perp, inv_inertia2, mass2, rad2perp, sfric, dfric, j, rel_vel, info.cn);
        impulse += info.cn * j - fj;

        float cps_ctr = (float)info.cps.size();
        rb1.velocity -= impulse / rb1.mass;
        rb1.angular_velocity += cross(impulse, rad1) * inv_inertia1;
        rb2.velocity += impulse / rb2.mass;
        rb2.angular_velocity -= cross(impulse, rad2) * inv_inertia2;
}
float DefaultSolver::getReactImpulse(const vec2f& rad1perp, float p1inv_inertia, float mass1, const vec2f& rad2perp, float p2inv_inertia, float mass2, 
        float restitution, const vec2f& rel_vel, vec2f cn) {
    float contact_vel_mag = dot(rel_vel, cn);
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
CollisionInfo DefaultSolver::detect(Transform* trans1, Collider* col1, Transform* trans2, Collider* col2) {
    CollisionInfo man;
    //ik its ugly but switch case will catch new variants if eCollisionShape will be getting more shapes
    switch(col1->type) {
        case eCollisionShape::Polygon:
            switch(col2->type) {
                case eCollisionShape::Polygon:
                    man = detectOverlap(col1->getPolygonShape(*trans1), col2->getPolygonShape(*trans2));
                break;
                case eCollisionShape::Circle:
                    man = detectOverlap(col2->getCircleShape(*trans2), col1->getPolygonShape(*trans1));
                    man.swapped = true;
                break;
                case eCollisionShape::Ray:
                    man = detectOverlap(col1->getPolygonShape(*trans1), col2->getRayShape(*trans2));
                break;
            }
        break;
        case eCollisionShape::Circle:
            switch(col2->type) {
                case eCollisionShape::Polygon:
                    man = detectOverlap(col1->getCircleShape(*trans1), col2->getPolygonShape(*trans2));
                break;
                case eCollisionShape::Circle:
                    man = detectOverlap(col1->getCircleShape(*trans1), col2->getCircleShape(*trans2));
                break;
                case eCollisionShape::Ray:
                    man = detectOverlap(col1->getCircleShape(*trans1), col2->getRayShape(*trans2));
                break;
            }
        break;
        case eCollisionShape::Ray: {
            switch (col2->type) {
                case eCollisionShape::Polygon:
                    man = detectOverlap(col2->getPolygonShape(*trans2), col1->getRayShape(*trans1));
                    man.swapped = true;
                break;
                case eCollisionShape::Circle:
                    man = detectOverlap(col2->getCircleShape(*trans2), col1->getRayShape(*trans1));
                    man.swapped = true;
                break;
                case eCollisionShape::Ray:
                    Log(LogLevel::ERROR) << "ray and ray should not be colliding";
                break;
            }
        }break;
    }
    return man;
}
CollisionInfo DefaultSolver::solve(RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction)  {
    auto man = detect(rb1.transform, rb1.collider, rb2.transform, rb2.collider);
    if(!man.detected) {
        return man;
    }
    handleOverlap(rb1, rb2, man);
    processReaction(man, rb1, rb2, restitution, sfriction, dfriction);
    return man;
}

}
