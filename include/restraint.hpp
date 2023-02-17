#pragma once
#include "col_utils.hpp"
#include "rigidbody.hpp"
#include <vector>

namespace EPI_NAMESPACE {

struct RestraintInterface {
    virtual void update(float delT) = 0;
    virtual std::vector<Rigidbody*> getRestrainedObjects() const = 0;
    virtual ~RestraintInterface() {}
};

struct RestraintDistance : public RestraintInterface {
    static constexpr float restraint_force = 1000.f;
    Rigidbody* a;
    Rigidbody* b;
    float dist = 0.f;
    void update(float delT) override;
    std::vector<Rigidbody*> getRestrainedObjects() const override {
        return {a, b};
    }

    RestraintDistance(float distance, Rigidbody* r1, Rigidbody* r2) : a(r1), b(r2), dist(distance) { }
};
struct RestraintPoint : public RestraintInterface {
    static constexpr float restraint_force = 1000.f;
    Rigidbody* a;
    vec2f model_point_a;
    Rigidbody* b;
    vec2f model_point_b;
    float dist = 0.f;
    void update(float delT) override;
    std::vector<Rigidbody*> getRestrainedObjects() const override {
        return {(Rigidbody*)a, (Rigidbody*)b};
    }
    RestraintPoint(float distance, Rigidbody* r1, vec2f model_pa, Rigidbody* r2, vec2f model_pb)
        : a(r1), model_point_a(model_pa), b(r2), model_point_b(model_pb), dist(distance) { }
};
struct RestraintRotation : public RestraintInterface {
    Rigidbody* a;
    float dist;
    Rigidbody* b;
    float a_angle;
    float b_angle;
    float con_angle;

    void update(float delT) override;
    std::vector<Rigidbody*> getRestrainedObjects() const override {
        return {a, b};
    }
    RestraintRotation(Rigidbody* r1, Rigidbody* r2) : a(r1), b(r2) {
        auto dir = b->getCollider().getPos() - a->getCollider().getPos();
        dist = len(dir);
        con_angle = atan2(dir.y, dir.x);
        a_angle = con_angle - a->getCollider().getRot();
        a_angle = fmod( a_angle, M_PI * 2.f);
        b_angle = con_angle - b->getCollider().getRot();
        b_angle = fmod( b_angle, M_PI * 2.f);
        std::cerr << a_angle << "\n";
        std::cerr << b_angle << "\n";
    };
};

}
