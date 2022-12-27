#include "types.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/System/Vector3.hpp"
#include <cmath>
#include <math.h>
#include <numeric>
namespace EPI_NAMESPACE {
vec2f norm(vec2f v) {
    float l = len(v);
    if(l == 0.f)
        return v;
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
float cross(vec2f a, vec2f b) {
    return a.x * b.y - b.x * a.y;
}
vec2f sign(vec2f x) {
    return { std::copysign(1.f, x.x), std::copysign(1.f, x.y) };
}
AABB AABBfromCircle(const Circle& c) {
    return AABB(c.pos - vec2f(c.radius, c.radius), c.pos + vec2f(c.radius, c.radius));
}
AABB AABBmm(vec2f min, vec2f max) {
    AABB a;
    a.min = min;
    a.max = max;
    return a;
}
AABB AABBcs(vec2f center, vec2f size) {
    AABB a;
    a.setCenter(center);
    a.setSize(size);
    return a;
}

Ray Rayab(vec2f a, vec2f b) {
    Ray r;
    r.pos = a;
    r.dir = b - a;
    return r;
}
Ray Raypd(vec2f p, vec2f d) {
    Ray r;
    r.pos = p;
    r.dir = d;
    return r;
}
void draw(sf::RenderWindow& rw, const Polygon& poly, clr_t clr) {
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
void drawFill(sf::RenderWindow& rw, const Polygon& poly, clr_t clr) {
    struct VertPair {
        sf::Vertex a;
        sf::Vertex b;
    };
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
Polygon PolygonReg(vec2f pos, float rot, size_t count, float dist) {
    std::vector<vec2f> model;
    for(size_t i = 0; i < count; i++) {
        model.push_back(vec2f(sinf(3.141f * 2.f * ((float)i / (float)count)), cosf(3.141f * 2.f * ((float)i / (float)count))) * dist );
    }
    return Polygon(pos, rot, model);
}
Polygon PolygonPoints(std::vector<vec2f> verticies) {
    vec2f avg = std::reduce(verticies.begin(), verticies.end()) / (float)verticies.size();
    for(auto& v : verticies)
        v -= avg;
    return Polygon(avg, 0.f, verticies);
}


vec2f operator* (vec2f a, vec2f b) {
    return vec2f(a.x * b.x, a.y * b.y);
}
}
