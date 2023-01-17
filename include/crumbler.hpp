#pragma once
#include "rigidbody.hpp"
#include "types.hpp"
#include "physics_manager.hpp"
#include "RNG.h"
#include <vector>

namespace EPI_NAMESPACE {
class Crumbler {
    struct VertNode {
        vec2f pos;
        bool isIncluded = false;
    };
    typedef std::vector<VertNode*> CenterNode;
public:
    RNG rng;
    float seg_size;
    float deviation;
    size_t reg_shape = 4U;
    std::vector<Polygon> crumble(Rigidbody* rb);
    Crumbler(float seg_size_, float deviation_) 
        : seg_size(seg_size_), deviation(deviation_) {}
};

}
