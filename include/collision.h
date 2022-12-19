#pragma once
#include "col_utils.h"
#include "rigidbody.hpp"
#include "utils.h"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace EPI_NAMESPACE {

vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
        float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
        float restitution, const vec2f& rel_vel, vec2f cn);

void processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
       vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, std::vector<vec2f> cps);
void processReaction(const CollisionManifold& maninfold, float bounce, float sfric, float dfric);


bool possibleOverlap(const Polygon& r1, const Polygon& r2);
bool detect(const Polygon &r1, const Polygon &r2, vec2f* cn = nullptr, float* t = nullptr);
CollisionManifold handleOverlap(RigidPolygon& r1, RigidPolygon& r2);

bool possibleOverlap(const Circle& r1, const Polygon& r2);
bool detect(const Circle &c, const Polygon &r, vec2f* cn = nullptr, float* overlap = nullptr, vec2f* cp = nullptr);
CollisionManifold handleOverlap(RigidCircle& c, RigidPolygon& r);

bool possibleOverlap(const Circle& r1, const Circle& r2);
bool detect(const Circle&c1, const Circle &c2, vec2f* cn = nullptr, float* t = nullptr, vec2f* cp = nullptr);
CollisionManifold handleOverlap(RigidCircle& c1, RigidCircle& c2);
bool handle(const CollisionManifold& manifold, float restitution, float sfriction, float dfriction);
}
