#include "col_utils.h"
#include "types.hpp"
#include <cmath>
namespace EPI_NAMESPACE {
bool PointVAABB(const vec2f& p, const AABB& r) {
    return (p.x >= r.center().x - r.size().x / 2 && p.y > r.center().y - r.size().y / 2
        && p.x < r.center().x + r.size().x / 2 && p.y <= r.center().y + r.size().y / 2);
}
bool PointVCircle(const vec2f& p, const Circle& c) {
    return len(p - c.pos) <= c.radius;
}
bool PointVPoly(const vec2f& p, const Polygon& poly) {
    int i, j, c = 0;
    for (i = 0, j = poly.getVertecies().size() - 1; i < poly.getVertecies().size(); j = i++) {
        auto& vi = poly.getVertecies()[i];
        auto& vj = poly.getVertecies()[j];
        if ( ((vi.y>p.y) != (vj.y>p.y)) &&
             (p.x < (vj.x-vi.x) * (p.y-vi.y) / (vj.y-vi.y) + vi.x) )
               c = !c;
        }
    return c;
}
bool AABBvAABB(const AABB& r1, const AABB& r2) {
    return (
        r1.min.x <= r2.max.x &&
        r1.max.x >= r2.min.x &&
        r1.min.y <= r2.max.y &&
        r1.max.y >= r2.min.y);
}
bool RayVAABB(vec2f ray_origin, vec2f ray_dir,
    const AABB& target, float* t_hit_near,
    vec2f* contact_normal, vec2f* contact_point)
{
    vec2f invdir = { 1.0f / ray_dir.x, 1.0f / ray_dir.y };
    vec2f t_size = target.size();
    //VVVVVVVVVVVVV
    //if((int)target.size.y % 2 == 0 && target.pos.y > ray_origin.y)
    //t_size -= vec2f(0, 1);
    //^^^^^^^^^^^^^
    vec2f t_near = (target.center() - t_size / 2.f - ray_origin) * invdir;
    vec2f t_far = (target.center() + t_size / 2.f - ray_origin) * invdir;

    if (std::isnan(t_far.y) || std::isnan(t_far.x)) return false;
    if (std::isnan(t_near.y) || std::isnan(t_near.x)) return false;
    if (t_near.x > t_far.x) std::swap(t_near.x, t_far.x);
    if (t_near.y > t_far.y) std::swap(t_near.y, t_far.y);

    if (t_near.x > t_far.y || t_near.y > t_far.x) return false;
    float thn = std::max(t_near.x, t_near.y);
    if (t_hit_near)
        *t_hit_near = thn;
    float t_hit_far = std::min(t_far.x, t_far.y);

    if (t_hit_far < 0)
        return false;
    if(contact_point)
        *contact_point = ray_origin + ray_dir * thn;
    if (t_near.x > t_near.y && contact_normal) {
        if (invdir.x < 0)
            *contact_normal = { 1, 0 };
        else
            *contact_normal = { -1, 0 };
    } else if (t_near.x < t_near.y && contact_normal) {
        if (invdir.y < 0)
            *contact_normal = { 0, 1 };
        else
            *contact_normal = { 0, -1 };
    }
    return true;
}
bool RayVRay(vec2f ray0_origin, vec2f ray0_dir,
    vec2f ray1_origin, vec2f ray1_dir,
    vec2f & contact_point, float* t0_near, float* t1_near)
{
    if (ray0_origin == ray1_origin) {
        contact_point = ray0_origin;
        return true;
    }
    auto dx = ray1_origin.x - ray0_origin.x;
    auto dy = ray1_origin.y - ray0_origin.y;
    auto det = ray1_dir.x * ray0_dir.y - ray1_dir.y * ray0_dir.x;
    if (det != 0) { // near parallel line will yield noisy results
        float u = (dy * ray1_dir.x - dx * ray1_dir.y) / det;
        float v = (dy * ray0_dir.x - dx * ray0_dir.y) / det;
        if (u >= 0 && v >= 0) {
            if (t0_near)
                *t0_near = u;
            if (t1_near)
                *t1_near = v;
            contact_point = ray0_origin + ray0_dir * u;
            return true;
        }
    }
    return false;
}
vec2f ClosestPointOnRay(vec2f ray_origin, vec2f ray_dir, vec2f point) {
    float ray_dir_len = len(ray_dir);
    vec2f seg_v_unit = ray_dir / ray_dir_len;
    float proj = dot(point - ray_origin, seg_v_unit);
    if (proj <= 0)
        return ray_origin;
    if (proj >= ray_dir_len)
        return ray_origin + ray_dir;
    return seg_v_unit * proj + ray_origin;
}
vec2f findPointOnEdge(vec2f point, const Polygon& poly) {
    const vec2f& pos = poly.getPos();
    auto dir = norm(point - pos);
    vec2f closest(INFINITY, INFINITY);
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        vec2f a = poly.getVertecies()[i];
        vec2f b = poly.getVertecies()[(i + 1) % poly.getVertecies().size()];
        vec2f adir = b - a;
        vec2f cp;
        float t;
        float tf;
        if(RayVRay(a, adir, pos, dir, cp, &t, &tf) && t < 1.f && qlen(cp - pos) < qlen(closest - pos) ) {
            closest = cp;
        }
    }
    return closest;
}
#define VERY_SMALL_AMOUNT 0.0005f
bool nearlyEqual(float a, float b) {
    return abs(a - b) < VERY_SMALL_AMOUNT;
}
bool nearlyEqual(vec2f a, vec2f b) {
    return nearlyEqual(a.x, b.x) && nearlyEqual(a.y, b.y);
}
std::vector<vec2f> getContactPoints(Polygon& p1, Polygon& p2) {
    std::vector<vec2f> cps;
    Polygon* poly1 = &p1;
    Polygon* poly2 = &p2;
    float closest_dist = INFINITY;
    for(int j = 0; j< 2; j++) {
        if(j == 1){ 
            poly1 = &p2;
            poly2 = &p1;
        }
        for(size_t i = 0; i < poly1->getVertecies().size(); i++) {
            vec2f a1 = poly1->getVertecies()[i];
            vec2f b1 = poly1->getVertecies()[(i + 1) % poly1->getVertecies().size()];
            for(size_t ii = 0; ii < poly2->getVertecies().size(); ii++) {
                vec2f t = poly2->getVertecies()[ii];
                vec2f closest = ClosestPointOnRay(a1, b1 - a1, t);
                if(nearlyEqual(qlen(t - closest), closest_dist)) {
                    if(!nearlyEqual(cps.front(), a1) && !nearlyEqual(cps.front(), b1))
                        cps.push_back(t);
                }else if(qlen(t - closest) < closest_dist) {
                    closest_dist = qlen(t - closest);
                    cps = {t};
                }
            }
        }
    }
    return cps;
}
float calcArea(const std::vector<vec2f>& model) {
    double area = 0.0;
    // Calculate value of shoelace formula
    int j = model.size() - 1;
    for (int i = 0; i < model.size(); i++) {
        area += (model[j].x + model[i].x) * (model[j].y - model[i].y);
        j = i;  // j is previous vertex to i
    }

    return abs(area / 2.0);
}
}
