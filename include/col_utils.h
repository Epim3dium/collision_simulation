#pragma once
#include "utils.h"
namespace EPI_NAMESPACE {
    bool PointVAABB(const vec2f& p, const AABB& r) ;
    bool PointVCircle(const vec2f& p, const Circle& c);
    bool PointVPoly(const vec2f& p, const Polygon& poly);
    bool AABBvAABB(const AABB& r1, const AABB& r2);
    bool RayVAABB(vec2f ray_origin, vec2f ray_dir,
        const AABB& target, float* t_hit_near,
        vec2f* contact_normal, vec2f* contact_point);
    bool RayVRay(vec2f ray0_origin, vec2f ray0_dir,
        vec2f ray1_origin, vec2f ray1_dir,
        vec2f & contact_point, float* t0_near, float* t1_near);
    vec2f ClosestPointOnRay(vec2f ray_origin, vec2f ray_dir, vec2f point);
    vec2f findPointOnEdge(vec2f point, const Polygon& poly);
    std::vector<vec2f>getContactPoints(Polygon& r1, Polygon& r2);
    float calcArea(const std::vector<vec2f>& model);
}
