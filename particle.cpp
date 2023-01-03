#include "particle.hpp"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/RectangleShape.hpp"
#include "types.hpp"
namespace  EPI_NAMESPACE {
RNG epi::Particle::s_rng;
template<> 
void Particle::m_apply<Particle::PosInit>(Particle::PosInit i) {
    this->pos.x = s_rng.Random(i.min.x, i.max.x);
    this->pos.y = s_rng.Random(i.min.y, i.max.y);
}
template<> 
void Particle::m_apply<Particle::RotInit>(Particle::RotInit i) {
    this->rot = s_rng.Random(i.min, i.max);
}
template<> 
void Particle::m_apply<Particle::AngVelInit>(Particle::AngVelInit i) {
    this->ang_vel = s_rng.Random(i.min, i.max);
}
template<> 
void Particle::m_apply<Particle::VelMagInit>(Particle::VelMagInit i) {
    this->vel *= s_rng.Random(i.min, i.max);
}
template<> 
void Particle::m_apply<Particle::VelAngleInit>(Particle::VelAngleInit i) {
    float angle = s_rng.Random(i.min, i.max);
    auto tmpvel = this->vel;
    this->vel.x = cos(angle) * tmpvel.x - sin(angle) * tmpvel.y;
    this->vel.y = sin(angle) * tmpvel.x + cos(angle) * tmpvel.y;
}
template<> 
void Particle::m_apply<Particle::LifetimeInit>(Particle::LifetimeInit i) {
    this->lifetime = s_rng.Random(i.min, i.max);
}
template<> 
void Particle::m_apply<Particle::ColorVecInit>(Particle::ColorVecInit i) {
    this->color = i.possible_colors[s_rng.Random<size_t>(0U, i.possible_colors.size())];
}
template<> 
void Particle::m_apply<Particle::ShapeInit>(Particle::ShapeInit i) {
    shape.type = i.type;
    shape.model = i.model;
    shape.size = i.size;
    shape.radius = i.radius;
}
template<> 
void Particle::m_apply<Particle::VelFuncInit>(Particle::VelFuncInit i) {
    change.vel = i.func;
}
template<> 
void Particle::m_apply<Particle::AngVelFuncInit>(Particle::AngVelFuncInit i) {
    change.ang_vel = i.func;
}
template<> 
void Particle::m_apply<Particle::ColorFuncInit>(Particle::ColorFuncInit i) {
    change.color = i.func;
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
