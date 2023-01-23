#include "crumbler.hpp"
#include "col_utils.hpp"
#include "physics_manager.hpp"
#include "rigidbody.hpp"
#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <sys/_types/_size_t.h>
#include <vector>
#include <deque>

namespace EPI_NAMESPACE {

Polygon Crumbler::m_getShape(ColliderInterface& col) {
    switch(col.getType()) {
        case eCollisionShape::Polygon:
            return (ColliderPolygon&)col;
        case eCollisionShape::Circle:
            return PolygonReg(col.getPos(), 0.f, 16U, ((ColliderCircle&)col).radius);
    }
}
std::vector<std::vector<Crumbler::VertNode> > Crumbler::m_generateVerticies(AABB aabb) {
    std::vector<std::vector<VertNode> > verticies;
    float angle = m_rng->Random(-3.141f, 3.141f);
    for(float y = aabb.min.y; y <= aabb.max.y + seg_size; y += seg_size) {
        verticies.push_back({});
        for(float x = aabb.min.x; x <= aabb.max.x + seg_size; x += seg_size) {
            auto vec = rotateVec(vec2f(x, y) - aabb.center(), angle);
            verticies.back().push_back({vec + aabb.center()});
        }
    }
    return verticies;
}
std::vector<Crumbler::CenterNode> Crumbler::m_generateCenters(std::vector<std::vector<VertNode> >& verticies) {
    std::vector<CenterNode> centers;
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
    return centers;
}
std::vector<Polygon> Crumbler::crumble(ColliderInterface& col, RNG* rng) {
    RNG tmpRng;
    if(rng) {
        m_rng = rng;
    } else {
        m_rng = &tmpRng;
    }
    float focus = 1.f;
    float offset_mag = seg_size * focus;

    Polygon shape = m_getShape(col);

    auto aabb = AABBfromPolygon(shape);
    aabb.setSize(aabb.size() * 3.f);
    auto verticies = m_generateVerticies(aabb);
    auto centers = m_generateCenters(verticies);

    float big_area = area(shape.getModelVertecies());

    //apply random deviation
    for(auto& v : verticies) {
        for(auto& vv : v) {
            vv.pos += vec2f(m_rng->Random(-deviation / 2.f, deviation / 2.f), m_rng->Random(-deviation / 2.f, deviation / 2.f));

            float l = len(shape.getPos() - vv.pos);
            if(focus_point.x != INFINITY && !nearlyEqual(l, 0.f)) {
                float invl = std::clamp(1.f / l, 0.f, 1.f);
                auto f = std::clamp((1.f - invl) * offset_mag, 0.f, l);

                vv.pos += norm(shape.getPos() - vv.pos) * f;
            }
        }
    }
    //update active nodes
    for(auto& v : verticies) {
        for(auto& vv : v) {
            if(isOverlappingPointPoly(vv.pos, shape))
                vv.isIncluded = true;
        }
    }
    //snap to edges
    for(auto& v : verticies) {
        for(auto& vv : v) {
            if(vv.isIncluded)
                continue;
            float closest_dist = INFINITY;
            vec2f closest;
            auto prev = shape.getVertecies().back();
            for(auto& p : shape.getVertecies()) {
                auto t = findClosestPointOnRay(p, prev - p, vv.pos);
                if(len(vv.pos - t) < closest_dist) {
                    closest_dist = len(vv.pos - t);
                    closest = t;
                }
                prev = p;
            }
            if(closest_dist <= seg_size + deviation) {
                vv.pos = closest;
                vv.isIncluded = true;
            }
        }
    }
    std::vector<Polygon> result;
    //functions helpers
    auto hash = [](vec2i i) {
        const int p1 = 73856093;
        const int p2 = 19349663;
        return int(i.x * p1) ^ int(i.y * p2);
    };
    auto cmp = [&hash](vec2f a, vec2f b) { 
        vec2i ai = (vec2i)a;
        vec2i bi = (vec2i)b;
        return hash(ai) < hash(bi);
    };
    //end of functions
    for(auto& c : centers) {
        std::set<vec2f, decltype(cmp)> model(cmp);
        for(int i = 0; i < c.size(); i++) {
            if(c[i]->isIncluded) {
                model.emplace(c[i]->pos);
            }
        }
        auto t = PolygonfromPoints(std::vector<vec2f>(model.begin(), model.end()));
        if(model.size() > 2 && area(t.getModelVertecies()) > big_area * 0.001f) {
            result.push_back(t);
        }
    }
    return result;
}

}

