#pragma once
#include "col_utils.h"
#include "utils.h"
#include <cmath>
#include <cstddef>
#include <vector>
namespace EPI_NAMESPACE {
    extern std::vector<vec2f > m_cps;
    struct Material {
        float restitution = 0.0f;
        float sfriction = 0.4f;
        float dfriction = 0.4f;
        float air_drag = 0.01f;
    };
    struct Rigidbody {
        bool isStatic = false;
        vec2f vel;
        float ang_vel = 0.f;
        float mass = 1.f;
        Material mat;
        virtual float inertia() const { return INFINITY; };
    };

    float getInertia(vec2f pos, const std::vector<vec2f>& model, float mass);

    class RigidPolygon  : public Rigidbody, public Polygon {
    private:
        void onChange() {
            m_maxr = 0.f;
            for(auto& p : getVertecies())
                m_maxr = std::max(m_maxr, len(p - getPos()));
            //since getInertia doesnt work just assume everything is a circle
            m_inertia = getInertia(vec2f(0, 0), getModelVertecies(), mass);
        }
        float m_inertia;
        float m_maxr = 0.f;
    public:
        float inertia() const override {
            return m_inertia;
        }
        float getMaxR() const {
            return m_maxr;
        }
        void addForce(vec2f dir, vec2f cp);

        RigidPolygon(const Polygon& poly) : Polygon(poly) { onChange(); }
        RigidPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Polygon(pos_, rot_, model_) 
        { 
            onChange(); 
        }
    };
    struct RigidCircle : public Rigidbody, public Circle {
        public:
        float inertia() const override {
            return 0.25f * mass * radius * radius;
        }
        RigidCircle(const Circle& c) : Circle(c) {}
        RigidCircle(vec2f pos, float radius) : Circle(pos, radius) {}
    };
    vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
            float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
    float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
            float restitution, const vec2f& rel_vel, vec2f cn);

    void processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
           vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, vec2f cp);

    bool possibleIntersection(const Polygon& r1, const Polygon& r2);
    bool detect(const Polygon &r1, const Polygon &r2, vec2f* cn = nullptr, float* t = nullptr);
    void handle(RigidPolygon& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction);

    bool possibleIntersection(const Circle& r1, const Polygon& r2);
    bool detect(const Circle &c, const Polygon &r, vec2f* cn = nullptr, float* overlap = nullptr, vec2f* cp = nullptr);
    void handle(RigidCircle& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction);

    bool detect(const Circle&c1, const Circle &c2, vec2f* cn = nullptr, float* t = nullptr, vec2f* cp = nullptr);
    void handle(RigidCircle& r1, RigidCircle& r2, float restitution, float sfriction, float dfriction);

    bool detect(const vec2f& v, const Polygon &c2, vec2f* cn = nullptr, float* t = nullptr, vec2f* cp = nullptr);
    void handle(vec2f& pos, Rigidbody& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction);
}
