#pragma once

#include "types.hpp"
#include "RNG.h"

#include <functional>
#include <vector>

namespace EPI_NAMESPACE {
static inline float return_const_float(float val, float time) {
    return val;
}
static inline vec2f return_const_vec2f(vec2f val, float time) {
    return val;
}
static inline Color return_const_color(Color val, float time) {
    return val;
}

class Particle {
    static RNG s_rng;
    template<class T>
    void m_apply(T init_val);

    template<class T>
    void m_init(T i) {
        m_apply(i);
    }
    template<class T, class... Initializers>
    void m_init(T i, Initializers... other) {
        m_apply(i);
        m_init(other...);
    }

    enum class eShape {
        Circle,
        Polygon,
        Rectangle,
    };
public:
    struct PosInit;
    struct RotInit;
    struct AngVelInit;
    struct VelMagInit;
    struct VelAngleInit;
    struct LifetimeInit;
    struct ColorVecInit;
    struct ShapeInit;
    struct VelFuncInit;
    struct AngVelFuncInit;
    struct ColorFuncInit;

    template<class... Initializers>
    void init(Initializers... i) {
        pos = vec2f(0.f, 0.f);
        rot = 0.f;
        vel = vec2f(1.f, 0.f);
        ang_vel = 0.f;

        time_passed = 0.f;
        lifetime = 1.f;

        change.vel = return_const_vec2f;
        change.ang_vel = return_const_float;
        change.color = return_const_color;

        m_init(i...);
        isActive = true;
    }

    void update(float delT);
    void draw(Window& rw);

    struct {
        eShape type;
        vec2f size;
        float radius;
        std::vector<vec2f> model;
    }shape;

    struct {
        std::function<vec2f(vec2f, float)> vel;
        std::function<float(float, float)> ang_vel;
        std::function<Color(Color, float)> color;
    }change;

    bool isActive = false;
    float rot;
    vec2f pos;

    float time_passed;
    float lifetime;

    vec2f vel;
    float ang_vel;

    Color color;

    template<class Initializer>
    Particle(Initializer init) {
        init(init);
    }
    template<class... Initializers>
    Particle(Initializers... inits) {
        init(inits...);
    }
    Particle() {}
};
struct Particle::PosInit{
    vec2f min;
    vec2f max;
    PosInit(vec2f p) : min(p), max(p) {}
    PosInit(vec2f mi, vec2f ma) : min(mi), max(ma) {}
};
struct Particle::RotInit{
    float min;
    float max;
    RotInit(float mi, float ma) : min(mi), max(ma) {}
};
struct Particle::AngVelInit{
    float min;
    float max;
    AngVelInit(float mi, float ma) : min(mi), max(ma) {}
};
struct Particle::VelMagInit{
    float min;
    float max;
    VelMagInit(float mi, float ma) : min(mi), max(ma) {}
};
struct Particle::VelAngleInit{
    float min;
    float max;
    VelAngleInit(float mi, float ma) : min(mi), max(ma) {}
};
struct Particle::LifetimeInit {
    float min;
    float max;
    LifetimeInit(float mi, float ma) : min(mi), max(ma) {}
};
struct Particle::ColorVecInit {
    std::vector<Color> possible_colors;
    ColorVecInit(std::vector<Color> clrs) : possible_colors(clrs) {}
};
struct Particle::ShapeInit {
    eShape type;
    vec2f size;
    float radius;
    std::vector<vec2f> model;
    //intializing as rectangle
    ShapeInit(vec2f s):              size(s), type(eShape::Rectangle) {}
    //itnitializing as circle
    ShapeInit(float r):              radius(r), type(eShape::Circle) {}
    //itnitializing as polygon 
    ShapeInit(std::vector<vec2f> m): model(m), type(eShape::Polygon) {}
};
struct Particle::VelFuncInit {
    std::function<vec2f(vec2f, float)> func;
    VelFuncInit(std::function<vec2f(vec2f, float)> f) : func(f) {}
};
struct Particle::AngVelFuncInit {
    std::function<float(float, float)> func;
    AngVelFuncInit(std::function<float(float, float)> f) : func(f) {} 
};
struct Particle::ColorFuncInit {
    std::function<Color(Color, float)> func;
    ColorFuncInit(std::function<Color(Color, float)> f) : func(f) {}
};

}
