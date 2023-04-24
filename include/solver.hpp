#pragma once
#include "col_utils.hpp"

#include "collider.hpp"
#include "rigidbody.hpp"
#include "trigger.hpp"

#include "types.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace epi {

struct CollisionInfo {
    bool detected;
    vec2f cn;
    std::vector<vec2f> cps;
    float overlap;
    bool swapped = false;
};
class SolverInterface {
public:
    virtual CollisionInfo detect(Collider* col1, Collider* col2) = 0;
    virtual CollisionInfo solve(RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction) = 0;
};
class DefaultSolver : public SolverInterface {
private:
    static vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
            float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
    static float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
            float restitution, const vec2f& rel_vel, vec2f cn);

    static void processReaction(const CollisionInfo& info, const RigidManifold& rb1, 
           const RigidManifold& rb2,float bounce, float sfric, float dfric);
public:

    CollisionInfo detect(Collider* col1, Collider* col2) override;
    CollisionInfo solve(RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction) override;
};

}
