#include "types.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/System/Vector3.hpp"
#include <cmath>
#include <math.h>
#include <numeric>
#include <vector>
namespace epi {
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
vec2f proj(vec2f a, vec2f plane_norm) {
    return (dot(a, plane_norm) / dot(plane_norm, plane_norm)) * plane_norm;
}
float cross(vec2f a, vec2f b) {
    return a.x * b.y - b.x * a.y;
}
vec2f sign(vec2f x) {
    return { std::copysign(1.f, x.x), std::copysign(1.f, x.y) };
}
AABB AABB::CreateFromCircle(const Circle& c) {
    return AABB::CreateMinMax(c.pos - vec2f(c.radius, c.radius), c.pos + vec2f(c.radius, c.radius));
}
AABB AABB::CreateFromPolygon(const Polygon& p) {
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
void draw(sf::RenderWindow& rw, const Polygon& poly, Color clr) {
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
void drawFill(sf::RenderTarget& rw, const Polygon& poly, Color clr) {
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
void drawOutline(sf::RenderTarget& rw, const Polygon& poly, Color clr) {
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
Polygon Polygon::CreateRegular(vec2f pos, float rot, size_t count, float dist) {
    std::vector<vec2f> model;
    for(size_t i = 0; i < count; i++) {
        model.push_back(vec2f(sinf(3.141f * 2.f * ((float)i / (float)count)), cosf(3.141f * 2.f * ((float)i / (float)count))) * dist );
    }
    return Polygon(pos, rot, model);
}
Polygon Polygon::CreateFromPoints(std::vector<vec2f> verticies) {
    vec2f avg = std::reduce(verticies.begin(), verticies.end()) / (float)verticies.size();
    for(auto& v : verticies)
        v -= avg;
    return Polygon(avg, 0.f, verticies);
}
Polygon Polygon::CreateFromAABB(const AABB& aabb) {
    std::vector<vec2f> points = {aabb.min, vec2f(aabb.min.x, aabb.max.y), aabb.max, vec2f(aabb.max.x, aabb.min.y)};
    return Polygon::CreateFromPoints(points);
}


vec2f operator* (vec2f a, vec2f b) {
    return vec2f(a.x * b.x, a.y * b.y);
}
using epi::Signal::Observer;
using epi::Signal::Subject;
void Observer::_removeSubject(const Subject* s) {
    auto itr =std::find(_subjects_observed.begin(), _subjects_observed.end(), s);
    if(itr != _subjects_observed.end()) {
        _subjects_observed.erase(itr);
    }
}
Observer::~Observer() {
    for(auto s : _subjects_observed) {
        s->removeObserver(this);
    }
}
void Subject::addObserver(Signal::Observer* observer) {
    _observers.push_back(observer);
    observer->_subjects_observed.push_back(this);
}
void Subject::removeObserver(Signal::Observer* observer) {
    auto itr = std::find(_observers.begin(), _observers.end(), observer);
    assert(itr != _observers.end());
    _observers.erase(itr);
}
void Subject::notify(const Subject& entity, Signal::Event event) {
    for (auto o : _observers) {
        o->onNotify(entity, event);
    }
}
Subject::~Subject() {
    for(auto o : _observers) {
        o->_removeSubject(this);
    }
}

}
