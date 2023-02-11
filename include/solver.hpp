#pragma once
#include "col_utils.hpp"

#include "rigidbody.hpp"
#include "trigger.hpp"

#include "types.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace EPI_NAMESPACE {

struct CollisionManifold {
    bool detected;
    Rigidbody* r1;
    Rigidbody* r2;

    vec2f r1pos;
    vec2f r2pos;

    vec2f cn;
    std::vector<vec2f> cps;
    float overlap;
};
class SolverInterface {
public:
    virtual CollisionManifold detect(ColliderInterface* col1, ColliderInterface* col2, Rigidbody* rb1, Rigidbody* rb2) = 0;
    virtual CollisionManifold solve(Rigidbody* rb1, Rigidbody* rb2, float restitution, float sfriction, float dfriction) = 0;
};
class DefaultSolver : public SolverInterface {
private:
    static vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
            float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
    static float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
            float restitution, const vec2f& rel_vel, vec2f cn);

    static void processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
           vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, std::vector<vec2f> cps);
    static void processReaction(const CollisionManifold& maninfold, float bounce, float sfric, float dfric);

    static bool handle(const CollisionManifold& manifold, float restitution, float sfriction, float dfriction);
public:

    CollisionManifold detect(ColliderInterface* col1, ColliderInterface* col2, Rigidbody* rb1 = nullptr, Rigidbody* rb2 = nullptr) override;
    CollisionManifold solve(Rigidbody* rb1, Rigidbody* rb2, float restitution, float sfriction, float dfriction) override;
};

}
