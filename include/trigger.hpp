#pragma once
#include "col_utils.hpp"
#include "collider.hpp"
#include "rigidbody.hpp"
#include "types.hpp"

namespace epi {
struct TriggerInterface {
private:
    Transform* _transform;
public:
    virtual Collider& getCollider() = 0;
    virtual void onActivation(Rigidbody* rb, vec2f cn) = 0;
    TriggerInterface(Transform* trans) : _transform(trans) {}
};
}
