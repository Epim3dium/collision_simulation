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

    enum class eShape {
        Circle,
        Polygon,
        Rectangle,
    };
public:
    struct InitInerface {
        virtual void apply(Particle& part) = 0;
        virtual ~InitInerface() {}
    };
    struct InitList {
    private:
        template<class InitT>
        void m_add(InitT first) {
            inits.push_back(new InitT(first));
        }
        template<class InitT, class ...InitOther>
        void m_add(InitT first, InitOther... rest) {
            inits.push_back(new InitT(first));
            m_add(rest...);
        }
    public:
        std::vector<InitInerface*> inits;
        template<class ...InitOther>
        InitList(InitOther... inits) {
            m_add(inits...);
        }
        ~InitList() {
            for(auto& i : inits) {
                delete i;
            }
        }
    };

    void init(const std::vector<InitInerface*>& init_values) {
        pos = vec2f(0.f, 0.f);
        rot = 0.f;
        vel = vec2f(1.f, 0.f);
        ang_vel = 0.f;

        time_passed = 0.f;
        lifetime = 1.f;

        change.vel = return_const_vec2f;
        change.ang_vel = return_const_float;
        change.color = return_const_color;

        for(auto& i : init_values) {
            i->apply(*this);
        }
        isActive = true;
    }
    void init(const InitList& init_values) {
        init(init_values.inits);
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

    Particle(std::vector<InitInerface*> inits) {
        init(inits);
    }
    Particle() {}
    struct PosInit : InitInerface {
        vec2f min;
        vec2f max;
        void apply(Particle& p) override;
        PosInit(vec2f p) : min(p), max(p) {}
        PosInit(vec2f mi, vec2f ma) : min(mi), max(ma) {}
    };
    struct RotInit : InitInerface {
        float min;
        float max;
        void apply(Particle& p) override;
        RotInit(float mi, float ma) : min(mi), max(ma) {}
    };
    struct AngVelInit : InitInerface {
        float min;
        float max;
        void apply(Particle& p) override;
        AngVelInit(float mi, float ma) : min(mi), max(ma) {}
    };
    struct VelMagInit : InitInerface {
        float min;
        float max;
        void apply(Particle& p) override;
        VelMagInit(float mi, float ma) : min(mi), max(ma) {}
    };
    struct VelAngleInit : InitInerface {
        float min;
        float max;
        void apply(Particle& p) override;
        VelAngleInit(float mi, float ma) : min(mi), max(ma) {}
    };
    struct LifetimeInit : InitInerface  {
        float min;
        float max;
        void apply(Particle& p) override;
        LifetimeInit(float mi, float ma) : min(mi), max(ma) {}
    };
    struct ColorVecInit : InitInerface  {
        std::vector<Color> possible_colors;
        void apply(Particle& p) override;
        ColorVecInit(std::vector<Color> clrs) : possible_colors(clrs) {}
    };
    struct ShapeInit : InitInerface  {
        eShape type;
        vec2f size;
        float radius;
        std::vector<vec2f> model;
        void apply(Particle& p) override;
        //intializing as rectangle
        ShapeInit(vec2f s):              size(s), type(eShape::Rectangle) {}
        //itnitializing as circle
        ShapeInit(float r):              radius(r), type(eShape::Circle) {}
        //itnitializing as polygon 
        ShapeInit(std::vector<vec2f> m): model(m), type(eShape::Polygon) {}
    };
    struct VelFuncInit : InitInerface  {
        std::function<vec2f(vec2f, float)> func;
        void apply(Particle& p) override;
        VelFuncInit(std::function<vec2f(vec2f, float)> f) : func(f) {}
    };
    struct AngVelFuncInit : InitInerface  {
        std::function<float(float, float)> func;
        void apply(Particle& p) override;
        AngVelFuncInit(std::function<float(float, float)> f) : func(f) {} 
    };
    struct ColorFuncInit : InitInerface  {
        std::function<Color(Color, float)> func;
        void apply(Particle& p) override;
        ColorFuncInit(std::function<Color(Color, float)> f) : func(f) {}
    };
};

}
