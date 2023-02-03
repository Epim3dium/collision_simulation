#include "rigidbody.hpp"
#include "solver.hpp"
#include "types.hpp"

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
    force /= mass;
    addVelocity(force, cp);
}
void RigidPolygon::addVelocity(vec2f vel, vec2f cp) {
    if(isStatic)
        return;
    vec2f cn = norm(vel);
    //convert angluar vel to linear
    vec2f rad = cp - getCollider().getPos();

    velocity += cn * dot(vel, norm(vel));
    if(!lockRotation)
        angular_velocity -= cross(vel, rad) / inertia(); 
}

}
