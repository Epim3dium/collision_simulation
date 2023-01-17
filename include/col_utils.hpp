#pragma once
#include "types.hpp"
namespace EPI_NAMESPACE {
vec2f rotateVec(vec2f vec, float angle);
bool PointVAABB(const vec2f& p, const AABB& r) ;
bool PointVCircle(const vec2f& p, const Circle& c);
bool PointVPoly(const vec2f& p, const Polygon& poly);
bool AABBvAABB(const AABB& r1, const AABB& r2);
bool AABBcontainsAABB(const AABB& r1, const AABB& r2);
bool RayVAABB(vec2f ray_origin, vec2f ray_dir,
    const AABB& target, float* t_hit_near = nullptr, float* t_hit_far = nullptr,
    vec2f* contact_normal = nullptr, vec2f* contact_point = nullptr);
bool RayVRay(vec2f ray0_origin, vec2f ray0_dir,
    vec2f ray1_origin, vec2f ray1_dir,
    vec2f & contact_point, float* t0_near = nullptr, float* t1_near = nullptr);
vec2f ClosestPointOnRay(vec2f ray_origin, vec2f ray_dir, vec2f point);
vec2f findPointOnEdge(vec2f point, const Polygon& poly);
void getContactPoints(Polygon& r1, Polygon& r2, std::vector<vec2f>& result);
float calcArea(const std::vector<vec2f>& model);

bool detect(const Polygon &r1, const Polygon &r2, vec2f* cn, float* t);
bool detect(const Circle &c, const Polygon &r, vec2f* cn, float* overlap, vec2f* cp);
bool detect(const Circle &c1, const Circle &c2, vec2f* cn, float* overlap, vec2f* cp);
}
