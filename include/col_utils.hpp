#pragma once
#include "types.hpp"
namespace EPI_NAMESPACE {

//rotates vetor with respect to the theta by angle in radians
vec2f rotateVec(vec2f vec, float angle);
//returns true if r1 contains the whole of r2
bool AABBcontainsAABB(const AABB& r1, const AABB& r2);
//finds the closest vector to point that lies on ray
vec2f findClosestPointOnRay(vec2f ray_origin, vec2f ray_dir, vec2f point);
//finds the closest vetor to point that lies on one of poly's edges
vec2f findClosestPointOnEdge(vec2f point, const Polygon& poly);
//returns all of contact points of 2 polygons
std::vector<vec2f> findContactPoints(Polygon& r1, Polygon& r2);
//calculates area of polygon whose center should be at {0, 0}
float area(const std::vector<vec2f>& model);
//returns true if a and b are nearly equal
bool nearlyEqual(float a, float b);
//returns true if a and b are nearly equal
bool nearlyEqual(vec2f a, vec2f b);

bool isOverlappingPointAABB(const vec2f& p, const AABB& r) ;
bool isOverlappingPointCircle(const vec2f& p, const Circle& c);
bool isOverlappingPointPoly(const vec2f& p, const Polygon& poly);
bool isOverlappingAABBAABB(const AABB& r1, const AABB& r2);

struct IntersectionRayAABBResult {
    bool detected;
    float time_hit_near;
    float time_hit_far;
    vec2f contact_normal;
    vec2f contact_point;
};
IntersectionRayAABBResult intersectRayAABB(vec2f ray_origin, vec2f ray_dir,
    const AABB& target);

struct IntersectionRayRayResult {
    bool detected;
    vec2f contact_point;
    float t_hit_near0;
    float t_hit_near1;
};
IntersectionRayRayResult intersectRayRay(vec2f ray0_origin, vec2f ray0_dir, vec2f ray1_origin, vec2f ray1_dir);

struct IntersectionPolygonPolygonResult {
    bool detected;
    vec2f contact_normal;
    float overlap;
};
IntersectionPolygonPolygonResult intersectPolygonPolygon(const Polygon &r1, const Polygon &r2);

struct IntersectionPolygonCircleResult {
    bool detected;
    vec2f contact_normal;
    vec2f contact_point;
    float overlap;
};
IntersectionPolygonCircleResult intersectCirclePolygon(const Circle &c, const Polygon &r);

typedef IntersectionPolygonCircleResult IntersectionCircleCircleResult;
IntersectionCircleCircleResult intersectCircleCircle(const Circle &c1, const Circle &c2);

}
