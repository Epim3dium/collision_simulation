#include "particle.hpp"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/RectangleShape.hpp"
#include "types.hpp"
namespace  epi {
RNG epi::Particle::s_rng;
void Particle::PosInit::apply(Particle& p) {
    p.pos.x = s_rng.Random(min.x, max.x);
    p.pos.y = s_rng.Random(min.y, max.y);
}
void Particle::RotInit::apply(Particle& p) {
    p.rot = s_rng.Random(min, max);
}
void Particle::AngVelInit::apply(Particle& p) {
    p.ang_vel = s_rng.Random(min, max);
}
void Particle::VelMagInit::apply(Particle& p) {
    p.vel *= s_rng.Random(min, max);
}
void Particle::VelAngleInit::apply(Particle& p) {
    float angle = s_rng.Random(min, max);
    auto tmpvel = p.vel;
    p.vel.x = cos(angle) * tmpvel.x - sin(angle) * tmpvel.y;
    p.vel.y = sin(angle) * tmpvel.x + cos(angle) * tmpvel.y;
}
void Particle::LifetimeInit::apply(Particle& p) {
    p.lifetime = s_rng.Random(min, max);
}
void Particle::ColorVecInit::apply(Particle& p) {
    p.color = possible_colors[s_rng.Random<size_t>(0U, possible_colors.size())];
}
void Particle::ShapeInit::apply(Particle& p) {
    p.shape.type   = type;
    p.shape.model  = model;
    p.shape.size   = size;
    p.shape.radius = radius;
}
void Particle::VelFuncInit::apply(Particle& p) {
    p.change.vel = func;
}
void Particle::AngVelFuncInit::apply(Particle& p) {
    p.change.ang_vel = func;
}
void Particle::ColorFuncInit::apply(Particle& p) {
    p.change.color = func;
}

void Particle::init(const std::vector<InitInerface*>& init_values) {
    pos = vec2f(0.f, 0.f);
    rot = 0.f;
    vel = vec2f(1.f, 0.f);
    ang_vel = 0.f;

    time_passed = 0.f;
    lifetime = 1.f;

    change.vel = helper::return_const_vec2f;
    change.ang_vel = helper::return_const_float;
    change.color = helper::return_const_color;

    for(auto& i : init_values) {
        i->apply(*this);
    }
    isActive = true;
}
void Particle::update(float delT) {
    time_passed += delT;
    float time = time_passed / lifetime;

    pos += change.vel(vel, time)* delT;
    rot += change.ang_vel(ang_vel, time)* delT;

    isActive = !(int)time;
}
void Particle::draw(Window& rw) {
    float time = time_passed / lifetime;
    Color clr = change.color(this->color, time);

    switch(shape.type) {
        case eShape::Circle: {
            sf::CircleShape c(shape.radius);
            c.setPosition(pos - vec2f(shape.radius, shape.radius));
            c.setFillColor(clr);
            c.setRotation(rot);
            rw.draw(c);
        } break;
        case eShape::Rectangle: {
            sf::RectangleShape r(shape.size);
            r.setPosition(pos - shape.size / 2.f);
            r.setFillColor(clr);
            r.setRotation(rot);
            rw.draw(r);
        } break;
        case eShape::Polygon: {
            Polygon poly(pos, rot, shape.model);
            drawFill(rw, poly, clr);
        } break;
    }
}
}
