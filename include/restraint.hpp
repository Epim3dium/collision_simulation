#pragma once
#include "col_utils.hpp"
#include "game_object.hpp"
#include "game_object_utils.hpp"
#include "rigidbody.hpp"
#include "transform.hpp"
#include <vector>

namespace epi {

struct Restraint : public GameObject {
    #define RESTRAINT_TYPE (typeid(Restraint).hash_code())
    Property getPropertyList() const override {
        return {RESTRAINT_TYPE, "restraint"};
    }
    virtual void update(float delT) = 0;
    virtual ~Restraint() {
        notify(*this, Signal::EventDestroyed);
    }
};

struct RestraintPointTrans : public Restraint {
    float damping_coef= 0.03f;

    RigidManifold a;
    Transform* trans;
    vec2f model_point_a;
    vec2f model_point_trans;
    float dist = 0.f;
    void update(float delT) override;
    RestraintPointTrans(RigidManifold m1, vec2f model_point, Transform* parent, vec2f model_parent)
        : a(m1), model_point_a(model_point), trans(parent), model_point_trans(model_parent) { }
};
struct RestraintRigidRigid: public Restraint {
    float damping_coef= 0.03f;

    RigidManifold a;
    RigidManifold b;
    vec2f model_point_a;
    vec2f model_point_b;
    float dist = 0.f;
    void update(float delT) override;
    RestraintRigidRigid(RigidManifold m1, vec2f model_point1, RigidManifold m2, vec2f model_point2)
        : a(m1), model_point_a(model_point1), b(m2), model_point_b(model_point2) { }
};

}
