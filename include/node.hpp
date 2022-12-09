#pragma once
#include "collision.h"
#include "utils.h"

#include <memory>
#include <vector>
namespace EPI_NAMESPACE {
    struct Connection;

    struct Node : public Rigidbody {
        vec2f pos;
        float radius = 5.f;
        std::vector< Connection* > cons;

        Node* getOtherNode(Connection& con);
        void forAllOthers(std::function<void(Node&, Connection&)> do_sth);

        Node(vec2f pos_, float mass_, bool isstatic = false) : pos(pos_) {
            mass = mass_; 
            isStatic = isstatic;
        }
        ~Node();
    };
    std::shared_ptr<Connection> createConnection(Node& n1, Node& n2, float len, float stren, vec2f n);

    struct Connection {
        Node* a;
        Node* b;

        float length;
        float strength = 1.f;
        float damp = 0.4f;
        vec2f normal;

        void update();
    };
}
