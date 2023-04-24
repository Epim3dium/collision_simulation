#pragma once
#include "col_utils.hpp"
#include "game_object_utils.hpp"
#include "imgui.h"
#include "transform.hpp"
#include "collider.hpp"
#include "material.hpp"

#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <sys/_types/_size_t.h>
#include <vector>
#include <set>

namespace epi {

/*
* a class holding all of tha basic rigidbody properties needed to compute collision response
*/
class Rigidbody : public GameObject {
public:

    bool isStatic = false;
    bool lockRotation = false;

    vec2f force;
    vec2f velocity;
    float angular_force = 0.f;
    float angular_velocity = 0.f;
    float mass = 1.f;
    float inertia = -1.f;


    float time_immobile = 0.f;

    #define RIGIDBODY_TYPE (typeid(Collider).hash_code())
    Property getPropertyList() const override {
        return {RIGIDBODY_TYPE, "rigidbody"};
    }
    bool isDormant() const {
        return time_immobile > 1.f || isStatic;
    }
    inline void addForce(vec2f force) {
        velocity += force / mass;
    }
    void addVelocity(vec2f dir, vec2f rad);
    void addForce(vec2f dir, vec2f rad);
    Rigidbody() {}
    ~Rigidbody() {
        this->notify(*this, Signal::EventDestroyed);
    }
};
struct RigidManifold {
    Transform* transform;
    Collider* collider;
    Rigidbody* rigidbody;
    Material* material;
    bool operator<(const RigidManifold& other) const {
        return transform > other.transform;
    }
    bool operator==(const RigidManifold& other) const {
        return transform == other.transform;
    }
};
}
