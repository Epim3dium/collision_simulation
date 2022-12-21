#pragma once
#include "col_utils.h"
#include "utils.h"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace EPI_NAMESPACE {
struct Rigidbody;
struct CollisionManifold {
    bool detected;
    Rigidbody* r1;
    Rigidbody* r2;

    vec2f r1pos;
    vec2f r2pos;

    vec2f cn;
    std::vector<vec2f> cps;
    float overlap;
};
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
    unsigned int debugFlag = false;
    bool isStatic = false;
    vec2f vel;
    float ang_vel = 0.f;
    float mass = 1.f;
    Material mat;
    Collider collider;

    virtual float inertia() const = 0;
    virtual void updateMovement(float delT = 1.f) = 0;
    virtual std::string getType() const = 0;
    //virtual bool detectPossibleOverlap(Rigidbody* other) = 0;
    virtual AABB aabb() const = 0;
    virtual CollisionManifold handleOverlap(Rigidbody* other) = 0;

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
    inline std::string getType() const  override  {
        return typeid(*this).name();
    }
    CollisionManifold handleOverlap(Rigidbody* other) override;
    AABB aabb() const override {
        return this->getAABB();
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
    inline std::string getType() const  override  {
        return typeid(*this).name();
    }
    AABB aabb() const override {
        return AABBfromCircle(*this);
    }
    //bool detectPossibleOverlap(Rigidbody* other) override;
    CollisionManifold handleOverlap(Rigidbody* other) override;
    RigidCircle(const Circle& c) : Circle(c) {}
    RigidCircle(vec2f pos, float radius) : Circle(pos, radius) {}
};
}
