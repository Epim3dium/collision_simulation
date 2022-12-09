#include "node.hpp"
#include "col_utils.h"
#include "collision.h"
#include <memory>

namespace EPI_NAMESPACE {
    Node* Node::getOtherNode(Connection& con) {
        if(con.a && con.a != this) {
            return con.a;
        } else if(con.b && con.b != this) {
            return con.b;
        }
        return nullptr;
    }
    void Node::forAllOthers(std::function<void(Node&, Connection&)> do_sth) {
        for(auto& c : cons) {
            if(!c || !c->a || !c->b)
                continue;
            auto other = getOtherNode(*c);
            if(other)
                do_sth(*other, *c); 
        }
    }
    std::shared_ptr<Connection> createConnection(Node& n1, Node& n2, float len, float stren, vec2f n) {
        Connection t;
        t.a = &n1;
        t.b = &n2;
        t.length = len;
        t.strength = stren;
        t.normal = n;
        auto result = std::make_shared<Connection>(t);
        n1.cons.push_back(result.get());
        n2.cons.push_back(result.get());
        return result;
    }
    void Connection::update() {
        float f = (length - len(a->pos - b->pos)) * strength / (a->mass + b->mass);
        auto n = norm(a->pos - b->pos);
        a->vel += n * f * (1.f - damp) * a->mass;
        b->vel -= n * f * (1.f - damp) * b->mass;
    }
    Node::~Node() {
        for(auto& c : cons) {
            Node*& me_in_con = c->a;
            if(c->b == this)
                me_in_con = c->b;
            me_in_con = nullptr;
        }
    }
}
