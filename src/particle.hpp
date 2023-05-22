#pragma once

#include "types.hpp"
#include "RNG.h"

#include <functional>
#include <vector>

namespace epi {
//default return functions for particle
namespace helper {
static inline float return_const_float(float val, float time) {
    return val;
}
static inline vec2f return_const_vec2f(vec2f val, float time) {
    return val;
}
static inline Color return_const_color(Color val, float time) {
    return val;
}

}

class Particle {
    static RNG s_rng;

    enum class eShape {
        Circle,
        Polygon,
        Rectangle,
    };
public:
    //any value initializing particle should have function that applies that initialization
    struct InitInerface {
        virtual void apply(Particle& part) = 0;
        virtual ~InitInerface() {}
    };
    //abstraction of the list of any initializers created in the future, only made to store pointer to interfaces
    //For example initializing particle with a list:
    //    Particle.init(Particle::InitList(Particle::PosInit(0.f, 0.f), Particle::RotInit(0.f)))
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

    //applying all initInterfaces provided in vector
    void init(const std::vector<InitInerface*>& init_values);
    void init(const InitList& init_values) {
        init(init_values.inits);
    }

    //moving and changing colors (applyng change functions)
    void update(float delT);
    //drawing shape
    void draw(Window& rw);

    struct {
        eShape type;
        vec2f size;
        float radius;
        std::vector<vec2f> model;
    }shape;

    //diffrent functions changing vel, ang_vel and color based on time to end of activation
    struct {
        std::function<vec2f(vec2f, float)> vel;
        std::function<float(float, float)> ang_vel;
        std::function<Color(Color, float)> color;
    }change;

    bool isActive = false;
    float rot;
    vec2f pos;

    //time passed since last initalization
    float time_passed;
    //max time allowed to live
    float lifetime;

    vec2f vel;
    float ang_vel;

    Color color;

    Particle(std::vector<InitInerface*> inits) {
        init(inits);
    }
    Particle() {}

    //list of initalizer structs
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
