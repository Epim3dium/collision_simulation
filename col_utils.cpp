#include "col_utils.hpp"
#include "types.hpp"
#include <cmath>
namespace EPI_NAMESPACE {

vec2f rotateVec(vec2f vec, float angle) {
    return vec2f(cos(angle) * vec.x - sin(angle) * vec.y,
        sin(angle) * vec.x + cos(angle) * vec.y);
}
#define SQR(x) ((x) * (x))
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
bool AABBcontainsAABB(const AABB& r1, const AABB& r2) {
    return (r2.min.x >= r1.min.x) && (r2.max.x <= r1.max.x) &&
				(r2.min.y >= r1.min.y) && (r2.max.y <= r1.max.y);
}
bool RayVAABB(vec2f ray_origin, vec2f ray_dir,
    const AABB& target, float* t_hit_near, float* t_hit_far,
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
    float thf = std::min(t_far.x, t_far.y);
    if (t_hit_far)
        *t_hit_far = thf;

    if (thf < 0)
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
void getContactPoints(Polygon& p1, Polygon& p2, std::vector<vec2f>& cps) {
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
                float dist = qlen(t - closest);
                if(dist == closest_dist) {
                    if(!nearlyEqual(cps.front(), a1) && !nearlyEqual(cps.front(), b1))
                        cps.push_back(t);
                }else if(dist < closest_dist) {
                    closest_dist = dist;
                    cps = {t};
                }
            }
        }
    }
}
float calcArea(const std::vector<vec2f>& model) {
    double area = 0.0;
    // Calculate value of shoelace formula
    for (int i = 0; i < model.size(); i++) {
      int i1 = (i + 1) % model.size();
      area += (model[i].y + model[i1].y) * (model[i1].x - model[i].x) / 2.0;
    }
    return abs(area / 2.0);
}
bool detect(const Polygon &r1, const Polygon &r2, vec2f* cn, float* t) {
    const Polygon *poly1 = &r1;
    const Polygon *poly2 = &r2;

    float overlap = INFINITY;
    
    for (int shape = 0; shape < 2; shape++) {
        if (shape == 1) {
            poly1 = &r2;
            poly2 = &r1;
        }
        for (int a = 0; a < poly1->getVertecies().size(); a++) {
            int b = (a + 1) % poly1->getVertecies().size();
            vec2f axisProj = { -(poly1->getVertecies()[b].y - poly1->getVertecies()[a].y), poly1->getVertecies()[b].x - poly1->getVertecies()[a].x };
            
            // Optional normalisation of projection axis enhances stability slightly
            float d = sqrtf(axisProj.x * axisProj.x + axisProj.y * axisProj.y);
            axisProj = { axisProj.x / d, axisProj.y / d };

            // Work out min and max 1D points for r1
            float min_r1 = INFINITY, max_r1 = -INFINITY;
            for (int p = 0; p < poly1->getVertecies().size(); p++) {
                float q = (poly1->getVertecies()[p].x * axisProj.x + poly1->getVertecies()[p].y * axisProj.y);
                min_r1 = std::min(min_r1, q);
                max_r1 = std::max(max_r1, q);
            }

            // Work out min and max 1D points for r2
            float min_r2 = INFINITY, max_r2 = -INFINITY;
            for (int p = 0; p < poly2->getVertecies().size(); p++) {
                float q = (poly2->getVertecies()[p].x * axisProj.x + poly2->getVertecies()[p].y * axisProj.y);
                min_r2 = std::min(min_r2, q);
                max_r2 = std::max(max_r2, q);
            }

            // Calculate actual overlap along projected axis, and store the minimum
            if(std::min(max_r1, max_r2) - std::max(min_r1, min_r2) < overlap) {
                overlap = std::min(max_r1, max_r2) - std::max(min_r1, min_r2);
                if(cn) {
                    *cn = axisProj;
                }
            }

            if (!(max_r2 >= min_r1 && max_r1 >= min_r2))
                return false;
        }
    }
    //correcting normal
    if(cn) {
        float d = dot(r2.getPos() - r1.getPos(), *cn);
        if(d > 0.f)
            *cn *= -1.f;
    }

    if(t)
        *t = overlap;
    return true;
}
bool detect(const Circle &c, const Polygon &r, vec2f* cn, float* overlap, vec2f* cp) {
    vec2f max_reach = c.pos + norm(r.getPos() - c.pos) * c.radius;

    vec2f closest(INFINITY, INFINITY);
    vec2f prev = r.getVertecies().back();
    for(const auto& p : r.getVertecies()) {
        vec2f tmp = ClosestPointOnRay(prev, p - prev, c.pos);
        if(qlen(closest - c.pos) > qlen(tmp - c.pos)) {
            closest = tmp;
        }
        prev = p;
    }
    bool isOverlapping = len(closest - c.pos) <= c.radius || PointVPoly(c.pos, r);
    if(!isOverlapping)
        return false;
    if(cn) {
        *cn = norm(c.pos - closest);
        if(dot(*cn, r.getPos() - closest) > 0.f) {
            *cn *= -1.f;
        }
    }
    if(overlap) {
        *overlap = c.radius - len(c.pos - closest);
    }
    if(cp) {
        *cp = closest;
    }
    return true;
}
bool detect(const Circle &c1, const Circle &c2, vec2f* cn, float* overlap, vec2f* cp) {
    vec2f dist = c1.pos - c2.pos;
    float dist_len = len(dist);
    if(dist_len > c1.radius + c2.radius) {
        return false;
    }
    if(cn)
        *cn = dist / dist_len;
    if(overlap)
        *overlap = c1.radius + c2.radius - dist_len;
    if(cp)
        *cp =  dist / dist_len * c2.radius + c2.pos;
    return true;
}
}
