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

    Polygon m_getShape(ColliderInterface& col);
    std::vector<std::vector<VertNode> > m_generateVerticies(AABB aabb);
    std::vector<CenterNode> m_generateCenters(std::vector<std::vector<VertNode> >& verticies);

    RNG* m_rng;
public:
    //amount of 'warping' towards focus_point when cracking
    float focus = 1.f;
    //warping focus point(if not set the warping will not take place)
    vec2f focus_point = {INFINITY, INFINITY};

    //segments into which the shape will be cut
    float seg_size;
    //maximum deviation of lines cutting the shape
    float deviation;
    /*
    * @param col is the collider that the user wants to split
    * @return vector of polygons aquired by splitting col into pieces
    */
    std::vector<Polygon> crumble(ColliderInterface& col, RNG* rng = nullptr);
    Crumbler(float seg_size_, float deviation_) 
        : seg_size(seg_size_), deviation(deviation_) {}
};

}
