#pragma once
#include "SFML/Graphics/RenderWindow.hpp"

#include "collision.h"
#include "node.hpp"
#include "utils.h"

#include <memory>
#include <vector>

namespace EPI_NAMESPACE {
    class SoftBody {
        std::vector<std::unique_ptr<Node>> m_nodes;
        std::vector<Node*> m_edge;
        float target_area = 0.f;

        std::vector<std::shared_ptr<Connection>> m_connections;
        vec2f m_getCenter();
        vec2f last_center = vec2f();
    public:
        vec2f center() const {
            return last_center;
        }
        const std::vector<std::unique_ptr<Node>>& getNodes() const {
            return m_nodes;
        }

        float grav = 0.1f;
        float drag_coef = 0.002f;
        vec2f force = vec2f(0, 0);

        float getArea(vec2f center);
        void update(const std::vector<RigidPolygon*>& environment);
        void updateNode(Node& n, const std::vector<RigidPolygon*>& environment);

        friend void drawFill(sf::RenderWindow& window, const SoftBody& sb, clr_t clr);
        SoftBody() {}
        SoftBody(Polygon shape, vec2i point_density, float mass);
    };
}
