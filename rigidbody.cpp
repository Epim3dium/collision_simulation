#include "rigidbody.hpp"
#include "collision.h"

namespace EPI_NAMESPACE {
//only dynamic objects should be placed as the r1 argument
//if r2 is static its mass in mat2 should be infinite
float getInertia(vec2f pos, const std::vector<vec2f>& model, float mass) {
    float area = 0;
    vec2f center = pos;
    float mmoi = 0;

    int prev = model.size()-1;
    for (int index = 0; index < model.size(); index++) {
        auto a = model[prev];
        auto b = model[index];

        float area_step = abs(cross(a, b))/2.f;
        float mmoi_step = area_step*(dot(a, a)+dot(b, b)+abs(dot(a, b)))/6.f;

        area += area_step;
        mmoi += mmoi_step;

        prev = index;
    }
    
    double density = mass/area;
    mmoi *= density;
    //mmoi -= mass * dot(center, center);
    if(std::isnan(mmoi)) {
        std::cerr << "mmoi calc erreor!";
        mmoi = 0.f;
    }
    return abs(mmoi);
}
void RigidPolygon::addForce(vec2f force, vec2f cp) {
    cp = findPointOnEdge(cp, *this);
    if(isStatic)
        return;
    force /= mass;
    vec2f cn = norm(force);
    //convert angluar vel to linear
    vec2f rad = cp - getPos();
    vec2f radperp(-rad.y, rad.x);

    vec2f pang_vel_lin = radperp * ang_vel;

    float rperp_dotN = dot(radperp, cn);
    //calculate relative velocity
    vec2f rel_vel = force -
        (vel + pang_vel_lin);

    vel += force;
    ang_vel -= cross(force, rad)/ inertia();
}
bool RigidPolygon::detectPossibleOverlap(Rigidbody *other) {
    if(other->getType() == typeid(RigidPolygon).name()) {
        return possibleOverlap(*this, *(RigidPolygon*)other);
    } else if(other->getType() == typeid(RigidCircle).name()) {
        return possibleOverlap(*(RigidCircle*)other, *this);
    }
    return false;
}
bool RigidCircle::detectPossibleOverlap(Rigidbody *other) {
    if(other->getType() == typeid(RigidPolygon).name()) {
        return possibleOverlap(*this, *(RigidPolygon*)other);
    } else if(other->getType() == typeid(RigidCircle).name()) {
        return possibleOverlap(*(RigidCircle*)other, *this);
    }
    return false;
}

CollisionManifold RigidPolygon::handleOverlap(Rigidbody *other) {
    if(other->getType() == typeid(RigidPolygon).name()) {
        return epi::handleOverlap(*this, *(RigidPolygon*)other);
    } else if(other->getType() == typeid(RigidCircle).name()) {
        return epi::handleOverlap(*(RigidCircle*)other, *this);
    }
    return {false};
}
CollisionManifold RigidCircle::handleOverlap(Rigidbody *other) {
    if(other->getType() == typeid(RigidPolygon).name()) {
        return epi::handleOverlap(*this, *(RigidPolygon*)other);
    } else if(other->getType() == typeid(RigidCircle).name()) {
        return epi::handleOverlap(*(RigidCircle*)other, *this);
    }
    return {false};
}
}