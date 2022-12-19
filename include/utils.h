#pragma once
#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>

#include "imgui.h"
#include "imgui-SFML.h"

#include "SFML/Graphics.hpp"

#define CONSOLAS_PATH "assets/Consolas.ttf"
#define EPI_NAMESPACE epi
namespace EPI_NAMESPACE {
    typedef sf::Vector2f vec2f;
    typedef sf::Vector2i vec2i;
    typedef sf::Color clr_t;

    namespace PastelColor {
        const clr_t bg =     clr_t(0xa89984ff);
        const clr_t bg1 =    clr_t(0xa89984ff);
        const clr_t bg2 =    clr_t(0xbdae93ff);
        const clr_t bg3 =    clr_t(0xebdbb2ff);
        const clr_t bg4 =    clr_t(0x1E3A4CFF);
        const clr_t Red =    clr_t(0x9d0006ff);
        const clr_t Green =  clr_t(0x79740eff);
        const clr_t Yellow = clr_t(0xb57614ff);
        const clr_t Blue =   clr_t(0x076678ff);
        const clr_t Purple = clr_t(0x8f3f71ff);
        const clr_t Aqua =   clr_t(0x427b58ff);
        const clr_t Orange = clr_t(0xaf3a03ff);
        const clr_t Gray =   clr_t(0x928374ff);
    };

    float len(vec2f);
    vec2f norm(vec2f);
    float qlen(vec2f);
    float dot(vec2f, vec2f);
    float cross(vec2f, vec2f);
    vec2f sign(vec2f);

    struct AABB {
        vec2f min;
        vec2f max;
        vec2f center() const {
            return min + (max-min) / 2.f;
        }
        vec2f size() const {
            return max - min;
        }
        void setCenter(vec2f c) {
            auto t = size();
            min = c - t / 2.f;
            max = c + t / 2.f;
        }
        void setSize(vec2f s) {
            auto t = center();
            min = t - s / 2.f;
            max = t + s / 2.f;
        }
        AABB() {}
        AABB(vec2f min_, vec2f max_) : min(min_), max(max_) {}
    };
    struct Ray {
        vec2f pos;
        vec2f dir;

        float length() const {
            return len(dir);
        }
        Ray() {}
    };
    struct Circle {

        vec2f pos;
        float radius;
        Circle() {}
        Circle(vec2f p, float r) : pos(p), radius(r) {}
    };
    class Polygon {
        std::vector<vec2f> points;
        std::vector<vec2f> model;
        float rotation;
        vec2f pos;
        AABB m_aabb;
        void m_updatePoints() {
            m_aabb.min = vec2f(INFINITY, INFINITY);
            m_aabb.max = vec2f(-INFINITY, -INFINITY);
            for(size_t i = 0; i < model.size(); i++) {
                const auto& t = model[i];
                points[i].x = t.x * cosf(rotation) - t.y * sinf(rotation);
                points[i].y = t.x * sinf(rotation) + t.y * cosf(rotation);
                points[i] += pos;

                m_aabb.min.x = std::min(m_aabb.min.x, points[i].x);
                m_aabb.max.x = std::max(m_aabb.max.x, points[i].x);
                m_aabb.min.y = std::min(m_aabb.min.y, points[i].y);
                m_aabb.max.y = std::max(m_aabb.max.y, points[i].y);
            }
        }
        void m_avgPoints() {
            vec2f avg = vec2f(0, 0);
            for(auto& p : model) {
                avg += p;
            }
            avg /= (float)model.size();
            for(auto& p : model) {
                p -= avg;
            }
        }
    public:
        AABB getAABB() const {
            return m_aabb;
        }
        float getRot() const {
            return rotation;
        }
        void setRot(float r) {
            rotation = r;
            m_updatePoints();
        }
        vec2f getPos() const {
            return pos;
        }
        void setPos(vec2f v) {
            pos = v;
            m_updatePoints();
        }

        const std::vector<vec2f>& getVertecies() const {
            return points;
        }
        const std::vector<vec2f>& getModelVertecies() const {
            return model;
        }
        Polygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : pos(pos_), rotation(rot_), model(model_), points(model_.size(), vec2f(0, 0)) {
            std::sort(model.begin(), model.end(), [](vec2f a, vec2f b) {return std::atan2(a.x, -a.y) > std::atan2(b.x, -b.y);});
            m_updatePoints();
            m_avgPoints();
        }
        friend void draw(sf::RenderWindow& rw, const Polygon& poly, clr_t clr);
        friend void drawFill(sf::RenderWindow& rw, const Polygon& poly, clr_t clr);
    };
    AABB AABBfromCircle(const Circle& c);

    AABB AABBmm(vec2f min, vec2f max);
    AABB AABBcs(vec2f center, vec2f size);

    Ray Rayab(vec2f a, vec2f b);
    Ray Raypd(vec2f p, vec2f d);

    Polygon PolygonReg(vec2f pos, float rot, size_t count, float dist);
    Polygon PolygonPoints(std::vector<vec2f> verticies);

    vec2f operator* (vec2f a, vec2f b);
}
