#include "types.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/System/Vector3.hpp"
#include "col_utils.hpp"

#include <cmath>
#include <math.h>
#include <numeric>
#include <vector>
namespace epi {
float angleAround(vec2f a, vec2f pivot, vec2f b) {
    return angle(a - pivot, b - pivot);
}
float angle(vec2f a, vec2f b) {
    return atan2(cross(a,b), dot(a,b));
}
vec2f norm(vec2f v) {
    float l = len(v);
    if(l == 0.f) {
        return v;
    }
    return v / l;
}
float len(vec2f v) {
    return sqrt(v.x * v.x + v.y * v.y);
}
float qlen(vec2f v) {
    return v.x * v.x + v.y * v.y;
}
float dot(vec2f a, vec2f b) {
    return a.x * b.x + a.y * b.y;
}
vec2f proj(vec2f a, vec2f plane_norm) {
    return (dot(a, plane_norm) / dot(plane_norm, plane_norm)) * plane_norm;
}
float cross(vec2f a, vec2f b) {
    return a.x * b.y - b.x * a.y;
}
vec2f sign(vec2f x) {
    return { std::copysign(1.f, x.x), std::copysign(1.f, x.y) };
}
vec2f rotateVec(vec2f vec, float angle) {
    return vec2f(cos(angle) * vec.x - sin(angle) * vec.y,
        sin(angle) * vec.x + cos(angle) * vec.y);
}
std::vector<ConvexPolygon> toTriangles(const std::vector<vec2f>& points) {
    std::vector<ConvexPolygon> result;
    std::vector<vec2f> tmp = points;
    while(result.size() != points.size() - 2) {
        size_t res_size_last = result.size();
        for(int i = 0; i < tmp.size(); i++) {
            auto first = tmp[i];
            auto mid_index = (i + 1) % tmp.size();
            auto mid = tmp[mid_index];
            auto last = tmp[(i + 2) % tmp.size()];
            float angle = angleAround(first, mid, last);
            if(angle < 0.f) {
                continue;
            }
            bool containsVertex = false;
            for(auto p : tmp) {
                if(p == first || p == mid || p == last)
                    continue;
                if(isOverlappingPointPoly(p, {first, mid, last})) {
                    containsVertex = true;
                    break;
                }
            }
            if(containsVertex) {
                continue;
            }
            result.push_back(ConvexPolygon::CreateFromPoints({first, mid, last}));
            tmp.erase(tmp.begin() + mid_index);
            break;
        }
        if(res_size_last == result.size()) {
            break;
        }
    }
    return result;
}
std::vector<ConvexPolygon> toBiggestConvexPolygons(const std::vector<ConvexPolygon>& polygons) {
    std::vector<bool> wasIncluded(polygons.size(), false);
    std::vector<ConvexPolygon> result;
    for(int i = 0; i < polygons.size(); i++) {
        if(wasIncluded[i])
            continue;
        wasIncluded[i] = true;
    }
    return result;
}
AABB AABB::CreateFromCircle(const Circle& c) {
    return AABB::CreateMinMax(c.pos - vec2f(c.radius, c.radius), c.pos + vec2f(c.radius, c.radius));
}
AABB AABB::CreateFromPolygon(const ConvexPolygon& p) {
    vec2f min = {INFINITY, INFINITY};
    vec2f max = {-INFINITY, -INFINITY};
    for(auto& v : p.getVertecies()) {
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
    }
    return AABB::CreateMinMax(min, max);
}
AABB AABB::CreateMinMax(vec2f min, vec2f max) {
    AABB a;
    a.min = min;
    a.max = max;
    return a;
}
AABB AABB::CreateCenterSize(vec2f center, vec2f size) {
    AABB a;
    a.setCenter(center);
    a.setSize(size);
    return a;
}
AABB AABB::CreateMinSize(vec2f min, vec2f size) {
    AABB a;
    a.min = min;
    a.max = a.min + size;
    return a;
}
Ray Ray::CreatePoints(vec2f a, vec2f b) {
    Ray r;
    r.pos = a;
    r.dir = b - a;
    return r;
}
Ray Ray::CreatePositionDirection(vec2f p, vec2f d) {
    Ray r;
    r.pos = p;
    r.dir = d;
    return r;
}
void draw(sf::RenderWindow& rw, const ConvexPolygon& poly, Color clr) {
    struct VertPair {
        sf::Vertex a;
        sf::Vertex b;
    };
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        sf::Vertex t[2];
        t[0].color = clr;
        t[1].color = clr;
        t[0].position = poly.getVertecies()[i];
        if(i != poly.getVertecies().size() - 1) {
            t[1].position = poly.getVertecies()[i + 1];
        } else {
            t[1].position = poly.getVertecies()[0];
        }
        rw.draw(t, 2, sf::Lines);
    }
}
void drawOutline(sf::RenderTarget& rw, const AABB& aabb, Color clr) {
    sf::Vertex t[5];
    vec2f vert[] = {aabb.bl(), aabb.br(), aabb.tr(), aabb.tl(), aabb.bl()};
    for(int i = 0; i < 5; i++) {
        t[i].color = clr;
        t[i].position = vert[i];
    }
    rw.draw(&t[0], 2, sf::Lines);
    rw.draw(&t[1], 2, sf::Lines);
    rw.draw(&t[2], 2, sf::Lines);
    rw.draw(&t[3], 2, sf::Lines);
}
void drawFill(sf::RenderTarget& rw, const AABB& aabb, Color clr) {
    sf::Vertex t[4];
    vec2f vert[] = {aabb.bl(), aabb.br(), aabb.tr(), aabb.tl()};
    for(int i = 0; i < 4; i++) {
        t[i].color = clr;
        t[i].position = vert[i];
    }
    rw.draw(t, 4, sf::Quads);
}
void drawFill(sf::RenderTarget& rw, const ConvexPolygon& poly, Color clr) {
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        sf::Vertex t[3];
        t[0].color = clr;
        t[1].color = clr;
        t[2].color = clr;
        t[0].position = poly.getVertecies()[i];
        t[2].position = poly.getPos();
        if(i != poly.getVertecies().size() - 1) {
            t[1].position = poly.getVertecies()[i + 1];
        } else {
            t[1].position = poly.getVertecies()[0];
        }
        rw.draw(t, 3, sf::Triangles);
    }
}
void drawOutline(sf::RenderTarget& rw, const ConvexPolygon& poly, Color clr) {
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        sf::Vertex t[2];
        t[0].color = clr;
        t[1].color = clr;
        t[0].position = poly.getVertecies()[i];
        if(i != poly.getVertecies().size() - 1) {
            t[1].position = poly.getVertecies()[i + 1];
        } else {
            t[1].position = poly.getVertecies()[0];
        }
        rw.draw(t, 2, sf::Lines);
    }
}
ConvexPolygon ConvexPolygon::CreateRegular(vec2f pos, float rot, size_t count, float dist) {
    std::vector<vec2f> model;
    for(size_t i = 0; i < count; i++) {
        model.push_back(vec2f(sinf(3.141f * 2.f * ((float)i / (float)count)), cosf(3.141f * 2.f * ((float)i / (float)count))) * dist );
    }
    return ConvexPolygon(pos, rot, model);
}
ConvexPolygon ConvexPolygon::CreateFromPoints(std::vector<vec2f> verticies) {
    vec2f avg = std::reduce(verticies.begin(), verticies.end()) / (float)verticies.size();
    for(auto& v : verticies)
        v -= avg;
    return ConvexPolygon(avg, 0.f, verticies);
}
ConvexPolygon ConvexPolygon::CreateFromAABB(const AABB& aabb) {
    std::vector<vec2f> points = {aabb.min, vec2f(aabb.min.x, aabb.max.y), aabb.max, vec2f(aabb.max.x, aabb.min.y)};
    return ConvexPolygon::CreateFromPoints(points);
}


vec2f operator* (vec2f a, vec2f b) {
    return vec2f(a.x * b.x, a.y * b.y);
}

}
