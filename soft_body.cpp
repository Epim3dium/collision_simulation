#include "soft_body.hpp"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/Vertex.hpp"

#include "col_utils.h"
#include "collision.h"
#include "utils.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>
namespace EPI_NAMESPACE {
    float SoftBody::getArea(vec2f area_center) {
        std::vector<vec2f> model;
        for(auto& p : m_edge) {
            model.push_back(p->pos - area_center);
        }
        return calcArea(model);
    }
    vec2f SoftBody::m_getCenter() {
        float total_mass = 0.f;
        vec2f total_pos(0, 0);
        for(auto& n : m_nodes) {
            total_pos += n->pos * n->mass;
            total_mass += n->mass;
        }
        return total_pos / total_mass;
    }

    void drawFill(sf::RenderWindow& window, const SoftBody& sb, clr_t clr) {
        auto prev = sb.m_edge.back();
        for(auto& p : sb.m_edge) {
            sf::Vertex v[3];
            v[0].position = p->pos;
            v[1].position = prev->pos;
            v[2].position = sb.last_center;

            v[0].color = clr;
            v[1].color = clr;
            v[2].color = clr;
            window.draw(v, 3U, sf::Triangles);
            prev = p;
        }

    }
    void SoftBody::update(const std::vector<RigidPolygon*>& environment) {
        for(auto& c : m_connections)
            c->update();
        vec2f center = m_getCenter();
        last_center = center;
        for(auto& e : m_edge) {
            e->vel += norm(e->pos - center) * 5.f;
        }
        for(auto& n : m_nodes) {
            n->vel.y += grav;

            float vel_len = len(n->vel);
            float drag_force = vel_len * vel_len * drag_coef;
            if(len(n->vel) > drag_force)
                n->vel -= norm(n->vel) * drag_force;
            updateNode(*n, environment);
        }

    }
    void SoftBody::updateNode(Node& n, const std::vector<RigidPolygon*>& environment) {
        if(!n.isStatic) {
            n.pos += n.vel;
            for(auto& p : environment) {
                handle(n.pos, n, *p, 0.f, 0.f, 0.f);
            }
        }
    }

    SoftBody::SoftBody(Polygon shape, vec2i density, float mass) {
        float angle = shape.getRot();
        shape.setRot(0.f);
        vec2f ratio = vec2f(shape.getAABB().size().x / (float)density.x, shape.getAABB().size().y / (float)density.y);

        vec2f size = shape.getAABB().size();
        vec2f shape_pos = shape.getPos();
        shape.setPos(size / 2.f);

        int w = 0;
        int h = 0;
        float point_mass = mass / density.x * density.y;
        for(float ty = 0; ty <= size.y; ty += ratio.y) {
            h++;
            w = 0;
            for(float tx = 0; tx <= size.x; tx += ratio.x) {
                w++;
                float x = tx - size.x / 2.f;
                float y = ty - size.y / 2.f;
                //if(PointVPoly(vec2f(tx, ty), shape)) {
                    m_nodes.push_back(std::make_unique<Node>(
                        Node(vec2f(cos(angle)*x - sin(angle) * y, sin(angle) * x + cos(angle) * y) + shape_pos, point_mass))
                    );
                //}
            }
        }

        float strength = 1.0f;
        float length = std::max(ratio.x, ratio.y);

        for(int i = 0; i < m_nodes.size(); i++) {
            int col = i % w;
            int row = i / w;
            if(col == 0 || row == 0 || col == w - 1 || row == h - 1) {
                m_edge.push_back(m_nodes[i].get());
            }
            if(col != 0) {
                auto* a = m_nodes[i].get();
                auto* b = m_nodes[i - 1].get();
                m_connections.push_back(createConnection(*a, *b, len(a->pos - b->pos), strength, vec2f(0, 0)));
            }
            if(row != 0) {
                auto* a = m_nodes[i].get();
                auto* b = m_nodes[i - w].get();
                m_connections.push_back(createConnection(*a, *b, len(a->pos - b->pos), strength, vec2f(0, 0)));
            }
            if(row != 0 && col != 0) {
                auto* a = m_nodes[i].get();
                auto* b = m_nodes[i - w - 1].get();
                m_connections.push_back(createConnection(*a, *b, len(a->pos - b->pos), strength, vec2f(0, 0)));
            }
            if(row != 0 && col != w - 1) {
                auto* a = m_nodes[i].get();
                auto* b = m_nodes[i - w + 1].get();
                m_connections.push_back(createConnection(*a, *b, len(a->pos - b->pos), strength, vec2f(0, 0)));
            }
        }
        std::sort(m_edge.begin(), m_edge.end(), 
                [&](Node* n1, Node* n2) {
                    vec2f pos1 = n1->pos - shape_pos;
                    vec2f pos2 = n2->pos - shape_pos;
                    return atan2(pos1.x, -pos1.y) > atan2(pos2.x, -pos2.y);
                });
        target_area = getArea(shape_pos);
    }
}
