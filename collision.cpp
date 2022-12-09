#include "collision.h"
#include "col_utils.h"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace EPI_NAMESPACE {
    bool possibleIntersection(const Polygon& r1, const Polygon& r2) {
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
    bool possibleIntersection(const Circle& r1, const Polygon& r2) {
        return len(r1.pos - r2.getPos()) < r1.radius + len(r2.getAABB().size());
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
    void processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
           vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, vec2f cp)
    {
        auto inertia1 = rb1.inertia();
        auto inertia2 = rb2.inertia();
        if(inertia1 != 0.f)
            inertia1 = 1.f / inertia1;

        if(inertia2 != 0.f)
            inertia2 = 1.f / inertia2;
        vec2f rad1 = cp - pos1;
        vec2f rad2 = cp - pos2;
        vec2f rad1perp(-rad1.y, rad1.x);
        vec2f rad2perp(-rad2.y, rad2.x);
        vec2f p1ang_vel_lin = rad1perp * rb1.ang_vel;
        vec2f p2ang_vel_lin = rad2perp * rb2.ang_vel;

        //calculate relative velocity
        vec2f rel_vel = (rb2.vel + p2ang_vel_lin) -
            (rb1.vel + p1ang_vel_lin);

        float j = getReactImpulse(rad1perp, inertia1, rb1.mass, rad2perp, inertia2, rb2.mass, bounce, rel_vel, cn);
        vec2f impulse = cn * j - 
            getFricImpulse(inertia1, rb1.mass, rad1perp, inertia2, rb2.mass, rad2perp, sfric, dfric, j, rel_vel, cn);

        if(!rb1.isStatic) {
            rb1.vel -= impulse / rb1.mass;

            rb1.ang_vel += cross(impulse, rad1) * inertia1;
        }
        if(!rb2.isStatic) {
            rb2.vel += impulse / rb2.mass;

            rb2.ang_vel -= cross(impulse, rad2) * inertia2;
        }
    }
    void handle(vec2f& pos, Rigidbody& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction) {
        if(r1.isStatic && r2.isStatic) {
            return;
        }
        vec2f cn;
        vec2f cp;
        float overlap;
        if(detect(pos, r2, &cn, &overlap, &cp)) {
            pos += cn * overlap;

            float j = dot(r1.vel, cn);
            j /= 1.f / r1.mass + 1.f / r2.mass;
            vec2f impulse = j * cn;
            if(!r1.isStatic)
                r1.vel -= impulse / r1.mass;

            if(!r2.isStatic) {
                r2.vel += impulse / r2.mass;
            }
        }
    }
    void handle(RigidCircle& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction) {
        if(r1.isStatic && r2.isStatic) {
            return;
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
            processReaction(r1.pos, r1, r1.mat, r2.getPos(), r2, r2.mat, restitution, sfriction, dfriction, cn, cp);
        }

    }
    void handle(RigidPolygon& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction) {
        if(r1.isStatic && r2.isStatic) {
            return;
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
            for(auto cp : cps)
                processReaction(r1.getPos(), r1, r1.mat, r2.getPos(), r2, r2.mat, restitution, sfriction, dfriction, cn, cp);
        }
    }
    void handle(RigidCircle& r1, RigidCircle& r2, float restitution, float sfriction, float dfriction) {
        if(r1.isStatic && r2.isStatic) {
            return;
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
            processReaction(r1.pos, r1, r1.mat, r2.pos, r2, r2.mat, restitution, sfriction, dfriction, cn, cp);
        }
    }
    void RigidPolygon::addForce(vec2f force, vec2f cp) {
        cp = findPointOnEdge(cp, *this);
        if(isStatic)
            return;
        force /= mass;
        vec2f cn = norm(force);
        //convert angluar vel to linear
        vec2f rad = cp - getPos();
        vec2f radperp(-rad.y, rad.x);

        vec2f pang_vel_lin = radperp * ang_vel;

        float rperp_dotN = dot(radperp, cn);
        //calculate relative velocity
        vec2f rel_vel = force -
            (vel + pang_vel_lin);

        vel += force;
        ang_vel -= cross(force, rad)/ inertia();
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
    vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
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
            (r1perp_dotT * r1perp_dotT) *  p1inertia +
            (r2perp_dotT * r2perp_dotT) *  p2inertia;

        float contact_vel_mag = dot(rel_vel, tangent);
        float jt = -contact_vel_mag;
        jt /= denom;

        vec2f friction_impulse;
        if(abs(jt) <= j * sfric)
            friction_impulse = tangent * jt;
        else
            friction_impulse = tangent * -j * dfric;
        return friction_impulse;
    }
}
