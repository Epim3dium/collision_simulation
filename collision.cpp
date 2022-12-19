#include "collision.h"
#include "col_utils.h"
#include "utils.h"
#include <algorithm>
#include <random>
#include <cmath>
#include <numeric>
#include <vector>


namespace EPI_NAMESPACE {
bool possibleOverlap(const Polygon& r1, const Polygon& r2) {
    return AABBvAABB(r1.getAABB(), r2.getAABB());
}
bool detect(const vec2f& v, const Polygon &p, vec2f* cn, float* t, vec2f* cp) {
    if(!PointVPoly(v, p))
        return false;
    vec2f prev = p.getVertecies().back();
    float dist = INFINITY;
    vec2f closest;
    for(auto p : p.getVertecies()) {
        auto t = ClosestPointOnRay(p, prev - p, v);
        if(qlen(closest - v) > qlen(t - v)) {
            closest = t;
        }
        prev = p;
    }
    if(cp)
        *cp = closest;
    if(t)
        *t = len(closest - v);
    if(cn)
        *cn = norm(closest - v);
    return true;
}
bool detect(const Polygon &r1, const Polygon &r2, vec2f* cn, float* t) {
    const Polygon *poly1 = &r1;
    const Polygon *poly2 = &r2;

    float overlap = INFINITY;
    
    for (int shape = 0; shape < 2; shape++)
    {
        if (shape == 1)
        {
            poly1 = &r2;
            poly2 = &r1;
        }

        for (int a = 0; a < poly1->getVertecies().size(); a++)
        {
            int b = (a + 1) % poly1->getVertecies().size();
            vec2f axisProj = { -(poly1->getVertecies()[b].y - poly1->getVertecies()[a].y), poly1->getVertecies()[b].x - poly1->getVertecies()[a].x };
            
            // Optional normalisation of projection axis enhances stability slightly
            float d = sqrtf(axisProj.x * axisProj.x + axisProj.y * axisProj.y);
            axisProj = { axisProj.x / d, axisProj.y / d };

            // Work out min and max 1D points for r1
            float min_r1 = INFINITY, max_r1 = -INFINITY;
            for (int p = 0; p < poly1->getVertecies().size(); p++)
            {
                float q = (poly1->getVertecies()[p].x * axisProj.x + poly1->getVertecies()[p].y * axisProj.y);
                min_r1 = std::min(min_r1, q);
                max_r1 = std::max(max_r1, q);
            }

            // Work out min and max 1D points for r2
            float min_r2 = INFINITY, max_r2 = -INFINITY;
            for (int p = 0; p < poly2->getVertecies().size(); p++)
            {
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

    // If we got here, the objects have collided, we will displace r1
    // by overlap along the vector between the two object centers
    if(t)
        *t = overlap;
    return true;
}
bool possibleOverlap(const Circle& r1, const Polygon& r2) {
    AABB circ_aabb(r1.pos - vec2f(r1.radius, r1.radius), r1.pos + vec2f(r1.radius, r1.radius));
    return AABBvAABB(circ_aabb, r2.getAABB());
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
#define SQR(x) ((x) * (x))
bool possibleOverlap(const Circle& r1, const Circle& r2) {
    return qlen(r1.pos - r2.pos) < SQR(r1.radius + r2.radius);
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
void processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
       vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, std::vector<vec2f> cps)
{
    float mass1 = rb1.isStatic ? INFINITY : rb1.mass;
    float mass2 = rb2.isStatic ? INFINITY : rb2.mass;
    float inv_inertia1 = rb1.isStatic ? INFINITY : rb1.inertia();
    float inv_inertia2 = rb2.isStatic ? INFINITY : rb2.inertia();
    if(inv_inertia1 != 0.f)
        inv_inertia1 = 1.f / inv_inertia1;
    if(inv_inertia2 != 0.f)
        inv_inertia2 = 1.f / inv_inertia2;


    vec2f impulse (0, 0);
    vec2f rb1vel(0, 0); float rb1ang_vel = 0.f;
    vec2f rb2vel(0, 0); float rb2ang_vel = 0.f;

    for(auto& cp : cps) {
        vec2f rad1 = cp - pos1;
        vec2f rad2 = cp - pos2;

        vec2f rad1perp(-rad1.y, rad1.x);
        vec2f rad2perp(-rad2.y, rad2.x);

        vec2f p1ang_vel_lin = rad1perp * rb1.ang_vel;
        vec2f p2ang_vel_lin = rad2perp * rb2.ang_vel;

        vec2f vel_sum1 = rb1.isStatic ? vec2f(0, 0) : rb1.vel + p1ang_vel_lin;
        vec2f vel_sum2 = rb2.isStatic ? vec2f(0, 0) : rb2.vel + p2ang_vel_lin;

        //calculate relative velocity
        vec2f rel_vel = vel_sum2 - vel_sum1;

        float j = getReactImpulse(rad1perp, inv_inertia1, mass1, rad2perp, inv_inertia2, mass2, bounce, rel_vel, cn);
        vec2f fj = getFricImpulse(inv_inertia1, mass1, rad1perp, inv_inertia2, mass2, rad2perp, sfric, dfric, j, rel_vel, cn);
        impulse += cn * j - fj;

        impulse /= (float)cps.size();
        if(!rb1.isStatic) {
            rb1vel -= impulse / rb1.mass;
            rb1ang_vel += cross(impulse, rad1) * inv_inertia1;
        }
        if(!rb2.isStatic) {
            rb2vel += impulse / rb2.mass;
            rb2ang_vel -= cross(impulse, rad2) * inv_inertia2;
        }
    }
    rb1.vel +=     rb1vel;
    rb1.ang_vel += rb1ang_vel;
    rb2.vel +=     rb2vel;
    rb2.ang_vel += rb2ang_vel;
}
void processReaction(const CollisionManifold& man, float bounce, float sfric, float dfric) {
    processReaction(man.r1pos, *man.r1, man.r1->mat, man.r2pos, *man.r2, man.r2->mat, bounce, sfric, dfric, man.cn, man.cps);
}
CollisionManifold handleOverlap(RigidCircle& r1, RigidPolygon& r2) {
    if(r1.isStatic && r2.isStatic) {
        return {false};
    }

    vec2f cn;
    vec2f cp;
    float overlap;

    if(detect(r1, r2, &cn, &overlap, &cp)) {
        if(r2.isStatic) {
            r1.pos += cn * overlap;
        } else if(r1.isStatic) {
            r2.setPos(r2.getPos() + -cn * overlap);
        } else {
            r1.pos += cn * overlap / 2.f;
            r2.setPos(r2.getPos() + -cn * overlap / 2.f);
        }
        //processReaction(r1.pos, r1, r1.mat, r2.getPos(), r2, r2.mat, restitution, sfriction, dfriction, cn, {cp});
        return {true, &r1, &r2, r1.pos, r2.getPos(), cn, {cp} , overlap};
    }
    return {false};
}
CollisionManifold handleOverlap(RigidPolygon& r1, RigidPolygon& r2) {
    if(r1.isStatic && r2.isStatic) {
        return {false};
    }
    vec2f cn;
    float overlap;
    auto cps = getContactPoints(r1, r2);

    if(detect(r1, r2, &cn, &overlap)) {
        if(r2.isStatic) {
            r1.setPos(r1.getPos() + cn * overlap);
        } else if(r1.isStatic) {
            r2.setPos(r2.getPos() + -cn * overlap);
        } else {
            r1.setPos(r1.getPos() + cn * overlap / 2.f);
            r2.setPos(r2.getPos() + -cn * overlap / 2.f);
        }
        //processReaction(r1.getPos(), r1, r1.mat, r2.getPos(), r2, r2.mat, restitution, sfriction, dfriction, cn, cps);
        return {true, &r1, &r2, r1.getPos(), r2.getPos(), cn, cps, overlap};
    }
    return {false};
}
CollisionManifold handleOverlap(RigidCircle& r1, RigidCircle& r2) {
    if(r1.isStatic && r2.isStatic) {
        return {false};
    }
    vec2f cn, cp;
    float overlap;

    if(detect(r1, r2, &cn, &overlap, &cp)) {
        if(r2.isStatic) {
            r1.pos += cn * overlap;
        } else if(r1.isStatic) {
            r2.pos += -cn * overlap;
        } else {
            r1.pos += cn * overlap / 2.f;
            r2.pos += -cn * overlap / 2.f;
        }
        //processReaction(r1.pos, r1, r1.mat, r2.pos, r2, r2.mat, restitution, sfriction, dfriction, cn, {cp});
        return {true, &r1, &r2, r1.pos, r2.pos, cn, {cp}, overlap};
    }
    return {false};
}
bool handle(const CollisionManifold& manifold, float restitution, float sfriction, float dfriction) {
    if(!manifold.detected)
        return false;
    processReaction(manifold.r1pos, *manifold.r1, manifold.r1->mat, manifold.r2pos, 
                    *manifold.r2, manifold.r2->mat, restitution, sfriction, dfriction, manifold.cn, manifold.cps);
    return true;
}
float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
        float restitution, const vec2f& rel_vel, vec2f cn) {
    float contact_vel_mag = dot(rel_vel, cn);
    if(contact_vel_mag < 0.f)
        return 0.f;
    //equation from net
    float r1perp_dotN = dot(rad1perp, cn);
    float r2perp_dotN = dot(rad2perp, cn);

    float denom = 1.f / mass1 + 1.f / mass2 +
        (r1perp_dotN * r1perp_dotN) *  p1inertia +
        (r2perp_dotN * r2perp_dotN) *  p2inertia;

    float j = -(1.f + restitution) * contact_vel_mag;
    j /= denom;
    return j;
}
vec2f getFricImpulse(float p1inv_inertia, float mass1, vec2f rad1perp, float p2inv_inertia, float mass2, const vec2f& rad2perp, 
        float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn) {
    vec2f tangent = rel_vel - cn * dot(rel_vel, cn);

    if(len(tangent) < 0.1f)
        return vec2f(0, 0);
    else
        tangent = norm(tangent);

    //equation from net
    float r1perp_dotT = dot(rad1perp, tangent);
    float r2perp_dotT = dot(rad2perp, tangent);

    float denom = 1.f / mass1 + 1.f / mass2 + 
        (r1perp_dotT * r1perp_dotT) *  p1inv_inertia +
        (r2perp_dotT * r2perp_dotT) *  p2inv_inertia;

    float contact_vel_mag = dot(rel_vel, tangent);
    float jt = -contact_vel_mag;
    jt /= denom;

    vec2f friction_impulse;
    if(abs(jt) <= j * sfric) {
        friction_impulse = tangent * jt;
    } else {
        friction_impulse = tangent * -j * dfric;
    }
    return friction_impulse;
}
}
