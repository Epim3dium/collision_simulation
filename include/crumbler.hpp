#pragma once
#include "rigidbody.hpp"
#include "types.hpp"
#include "physics_manager.hpp"
#include "RNG.h"
#include <cmath>
#include <vector>

namespace EPI_NAMESPACE {
class Crumbler {
    struct VertNode {
        vec2f pos;
        bool isIncluded = false;
    };
    typedef std::vector<VertNode*> CenterNode;
    Polygon m_getShape(Rigidbody* rb);
    std::vector<std::vector<VertNode> > m_generateVerticies(AABB aabb);
    std::vector<CenterNode> m_generateCenters(std::vector<std::vector<VertNode> >& verticies);

    RNG rng;
public:
    float focus = 1.f;
    vec2f focus_point = {INFINITY, INFINITY};

    float seg_size;
    float deviation;
    std::vector<Polygon> crumble(Rigidbody* rb);
    Crumbler(float seg_size_, float deviation_) 
        : seg_size(seg_size_), deviation(deviation_) {}
};

}
