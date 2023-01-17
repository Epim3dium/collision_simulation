#include "crumbler.hpp"
#include "col_utils.hpp"
#include "rigidbody.hpp"
#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <sys/_types/_size_t.h>
#include <vector>
#include <deque>

namespace EPI_NAMESPACE {

std::vector<Polygon> Crumbler::crumble(Rigidbody* rb) {
    std::vector<std::vector<VertNode> > verticies;
    std::vector<CenterNode> centers;

    AABB aabb = rb->aabb();
    std::deque<VertNode>* pervy = nullptr;
    for(float y = aabb.min.y; y <= aabb.max.y + seg_size; y += seg_size) {
        verticies.push_back({});
        for(float x = aabb.min.x; x <= aabb.max.x + seg_size; x += seg_size) {
            verticies.back().push_back({vec2f(x, y)});
        }
    }
    auto imax = verticies.size() - 1;
    auto iimax = verticies.back().size() - 1;
    for(int i = 0; i < imax; i++) {
        for(int ii = 0; ii < iimax; ii++) {
            centers.push_back( {} );
            auto& cur = centers.back();
            cur.push_back(&verticies[i][ii]);
            cur.push_back(&verticies[i + 1][ii]);
            cur.push_back(&verticies[i][ii + 1]);
            cur.push_back(&verticies[i + 1][ii + 1]);
        }
    }
    //apply random deviation
    for(auto& v : verticies) {
        for(auto& vv : v) {
            vv.pos += vec2f(rng.Random(-deviation / 2.f, deviation / 2.f), rng.Random(-deviation / 2.f, deviation / 2.f));
        }
    }
    Polygon shape;
    switch(rb->getType()) {
        case eRigidShape::Polygon:
            shape = *(RigidPolygon*)rb;
            break;
        case eRigidShape::Circle:
            shape = PolygonReg(rb->getPos(), 0.f, 24U, ((RigidCircle*)rb)->radius);
            break;
    }
    //snap to edges
    for(auto& v : verticies) {
        for(auto& vv : v) {
            float closest_dist = INFINITY;
            vec2f closest;
            auto prev = shape.getVertecies().back();
            for(auto& p : shape.getVertecies()) {
                auto t = ClosestPointOnRay(p, prev - p, vv.pos);
                if(len(vv.pos - t) < closest_dist) {
                    closest_dist = len(vv.pos - t);
                    closest = t;
                }
                prev = p;
            }
            if(closest_dist < seg_size) {
                vv.pos = closest;
                vv.isIncluded = true;
            }
        }
    }
    //update active nodes
    for(auto& v : verticies) {
        for(auto& vv : v) {
            if(PointVPoly(vv.pos, shape))
                vv.isIncluded = true;
        }
    }
    std::vector<Polygon> result;
    for(auto& c : centers) {
        std::vector<vec2f> model;
        for(int i = 0; i < c.size(); i++) {
            if(c[i]->isIncluded) {
                model.push_back(c[i]->pos);
            }
        }
        if(model.size() > 2)
            result.push_back(PolygonfromPoints(model));
    }
    return result;
}

}
