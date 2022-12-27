#pragma once
#include "col_utils.h"
#include "types.hpp"
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
    bool isDormant() const {return dormant_time > 120.f;}
    float dormant_time = 0;

    size_t layer;
    bool lockRotation = false;
    Rigidbody* now_colliding = nullptr;
    Collider() : layer(getNextLayer()) {}
};
struct Material {
    float restitution = 0.0f;
    float sfriction = 0.4f;
    float dfriction = 0.4f;
    float air_drag = 0.0001f;
};
class Rigidbody {
    inline static size_t getNextID()  {
        static size_t s_id = 1;
        return s_id++;
    }
    size_t m_id;
public:
    unsigned int debugFlag = false;
    bool isStatic = false;

    vec2f velocity;
    vec2f acceleration;

    float angular_velocity = 0.f;
    float mass = 1.f;
    Material material;
    Collider collider;

    //get inertia of custom shape
    virtual float inertia() const = 0;
    //update pos with valocity
    virtual void updateMovement(float delT = 1.f) = 0;
    //get type identificator
    virtual std::string getType() const = 0;
    
    //get axis aligned bounding box of custom shape
    virtual AABB aabb() const = 0;
    //resolve overlap function for rigidbodies with diffrent types
    virtual CollisionManifold handleOverlap(Rigidbody* other) = 0;

    inline void addForce(vec2f force) {
        velocity += force / mass;
    }
    inline size_t getID() const {
        return m_id;
    }
    Rigidbody() : m_id(getNextID()) {}
};

//calculating inertia of polygon shape
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
        setPos(getPos() + this->velocity * delT);
        if(!collider.lockRotation)
            setRot(getRot() + this->angular_velocity * delT);
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
        this->pos += velocity * delT;
        if(!collider.lockRotation)
            this->rot += angular_velocity * delT;
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
