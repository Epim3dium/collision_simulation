#pragma once
#include "col_utils.h"
#include "utils.h"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace EPI_NAMESPACE {
    struct Rigidbody;

    struct Collider {
    private:
        inline static size_t getNextLayer() {
            static size_t id = 0;
            return ++id;
        }
    public:
        size_t layer;
        bool isSleeping = false;
        Rigidbody* now_colliding = nullptr;
        Collider() : layer(getNextLayer()) {}
    };
    struct Material {
        float restitution = 0.0f;
        float sfriction = 0.4f;
        float dfriction = 0.4f;
        float air_drag = 0.0001f;
    };
#define MIN_VEL_TO_SLEEP 0.1f
#define MIN_ANG_VEL_TO_SLEEP 0.09f
#define MIN_TIME_DORMANT 15U
    class Rigidbody {
        inline static size_t getNextID()  {
            static size_t s_id = 1;
            return s_id++;
        }
        size_t m_id;
    public:
        bool isStatic = false;
        vec2f vel;
        float ang_vel = 0.f;
        float mass = 1.f;
        Material mat;
        Collider collider;

        virtual float inertia() const { return INFINITY; };
        virtual void updateMovement(float delT = 1.f) {}
        inline void addForce(vec2f force) {
            vel += force / mass;
        }

        inline size_t getID() const {
            return m_id;
        }
        Rigidbody() : m_id(getNextID()) {}
    };

    float getInertia(vec2f pos, const std::vector<vec2f>& model, float mass);

    class RigidPolygon  : public Rigidbody, public Polygon {
    private:
        float m_inertia;
        inline void onChange() {
            m_inertia = getInertia(vec2f(0, 0), getModelVertecies(), mass);
        }
    public:
        void addForce(vec2f dir, vec2f cp);
        inline float inertia() const override { 
            return m_inertia; 
        }
        inline void updateMovement(float delT) override {
            setPos(getPos() + this->vel * delT);
            setRot(getRot() + this->ang_vel * delT);
        }

        RigidPolygon(const Polygon& poly) : Polygon(poly) { 
            onChange(); 
        }
        RigidPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : Polygon(pos_, rot_, model_) { 
            onChange(); 
        }
    };
    struct RigidCircle : public Rigidbody, public Circle {
        public:
        float rot;
        inline float inertia() const override {
            return 0.25f * mass * radius * radius;
        }
        inline void updateMovement(float delT) override {
            this->pos += vel * delT;
            this->rot += ang_vel * delT;
        }
        RigidCircle(const Circle& c) : Circle(c) {}
        RigidCircle(vec2f pos, float radius) : Circle(pos, radius) {}
    };

    vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
            float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
    float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
            float restitution, const vec2f& rel_vel, vec2f cn);

    void processReaction(vec2f pos1, Rigidbody& rb1, const Material& mat1, 
           vec2f pos2, Rigidbody& rb2, const Material& mat2, float bounce, float sfric, float dfric, vec2f cn, std::vector<vec2f> cps);

    bool possibleIntersection(const Polygon& r1, const Polygon& r2);
    bool detect(const Polygon &r1, const Polygon &r2, vec2f* cn = nullptr, float* t = nullptr);
    bool handle(RigidPolygon& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction);

    bool possibleIntersection(const Circle& r1, const Polygon& r2);
    bool detect(const Circle &c, const Polygon &r, vec2f* cn = nullptr, float* overlap = nullptr, vec2f* cp = nullptr);
    bool handle(RigidCircle& r1, RigidPolygon& r2, float restitution, float sfriction, float dfriction);

    bool possibleIntersection(const Circle& r1, const Circle& r2);
    bool detect(const Circle&c1, const Circle &c2, vec2f* cn = nullptr, float* t = nullptr, vec2f* cp = nullptr);
    bool handle(RigidCircle& r1, RigidCircle& r2, float restitution, float sfriction, float dfriction);
}
