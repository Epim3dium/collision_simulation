#include "physics_manager.hpp"
#include "imgui.h"
#include "solver.hpp"
#include "rigidbody.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sys/_types/_size_t.h>
#include <vector>
#include <thread>
#include <mutex>

#define SQR(x) ((x) * (x))
#define MAX_VELOCITY 50.0f

namespace EPI_NAMESPACE {


typedef int hash_vec;
static hash_vec hash(vec2i pos) {
    const hash_vec a = 73856093;
    const hash_vec b = 83492791;
    return (pos.x * a) ^ (pos.y * b);
}
std::vector<int> hash(AABB shape, float seg_size) {
    vec2i min_hash(shape.min / seg_size);
    vec2i max_hash(shape.max / seg_size);
    std::vector<hash_vec> r;
    for(int y = min_hash.y; y <= max_hash.y; y++) {
        for(int x = min_hash.x; x <= max_hash.x; x++) {
            r.push_back(hash(vec2i(x, y)) );
        }
    }
    return r;
}

static bool areIncompatible(Rigidbody& rb1, Rigidbody& rb2) {
    return ( rb1.isStatic && rb2.isStatic) || 
        (rb1.collider.layer == rb2.collider.layer);
}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
    return m_rigidbodiesQT.findAllIntersections();
}
void PhysicsManager::processNarrowRange(std::vector<std::pair<Rigidbody*, Rigidbody*>>::const_iterator begining,std::vector<std::pair<Rigidbody*, Rigidbody*>>::const_iterator ending)
{
    for(auto ci = begining; ci != ending; ci++) {
        float restitution = m_selectFrom(ci->first->material.restitution, ci->second->material.restitution, bounciness_select);
        float sfriction = m_selectFrom(ci->first->material.sfriction, ci->second->material.sfriction, friction_select);
        float dfriction = m_selectFrom(ci->first->material.dfriction, ci->second->material.dfriction, friction_select);
        if(areIncompatible(*ci->first, *ci->second))
            continue;
        //ewewewewewewwwwww pls don, float delTt judge me
        auto result = m_solver->solve(ci->first, ci->second, restitution, sfriction, dfriction);
        if(result.detected) {
            ci->first->collider.pressure += result.overlap;
            ci->second->collider.pressure += result.overlap;
        }
    }
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    processNarrowRange(col_list.begin(), col_list.end());
}
void PhysicsManager::m_updateRestraints(float delT) {
    for(auto& r : m_restraints)
        r->update(delT);
}
void PhysicsManager::m_processTriggers() {
    for(auto& t : m_triggers) {
        if(t->aabb().min == t->aabb().max)
            continue;
        auto possible = m_rigidbodiesQT.query(t->aabb());
        for(auto& r : possible) {
            auto man = m_solver->detect(r, t);
            if(man.detected) {
                t->onActivation(r, man.contact_normal);
            }
        }
    }
}

void PhysicsManager::m_updateRigidbody(Rigidbody& rb, float delT) {
    //updating velocity and physics
    if(rb.isStatic)
        return;
    rb.velocity.y += grav * delT;
    if(qlen(rb.velocity) > 0.001f)
        rb.velocity -= norm(rb.velocity) * std::clamp(qlen(rb.velocity) * rb.material.air_drag, 0.f, len(rb.velocity)) * delT;
    if(abs(rb.angular_velocity) > 0.001f)
        rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * rb.material.air_drag, 0.f, abs(rb.angular_velocity)) * delT;

    rb.setPos(rb.getPos() + rb.velocity * delT);
    if(!rb.collider.lockRotation)
        rb.setRot(rb.getRot() + rb.angular_velocity * delT);
    rb.collider.last_pos = rb.getPos();
}
void PhysicsManager::m_updatePhysics(float delT) {
    for(auto r : m_rigidbodies) {
        m_updateRigidbody(*r, delT);
    }
}
void PhysicsManager::update(float delT ) {
    float deltaStep = delT / (float)steps;
    for(auto r : m_rigidbodies) {
        r->collider.now_colliding = nullptr;
        r->collider.pressure = 0.f;
    }
    for(int i = 0; i < steps; i++) {
        m_updateRestraints(deltaStep);

        for(auto r : m_rigidbodies)
            m_rigidbodiesQT.add(r);

        auto col_list = processBroadPhase();
        m_rigidbodiesQT.clear();

        m_updatePhysics(deltaStep);
        processNarrowPhase(col_list);

    }
    for(auto r : m_rigidbodies)
        m_rigidbodiesQT.add(r);
    m_processTriggers();
    m_rigidbodiesQT.clear();

    m_rigidbodiesQT.updateLeafes();
}
template<class T>
static void unbind_any(T obj, std::vector<T>& obj_vec) {
    auto itr = obj_vec.begin();
    for(; itr != obj_vec.end(); itr++) {
        if(*itr == obj)
            break;
    }
    if(itr != obj_vec.end())
        obj_vec.erase(itr);
}
void PhysicsManager::unbind(Rigidbody* rb) {
    unbind_any(rb, m_rigidbodies);
}
void PhysicsManager::unbind(RestraintInterface* res) {
    unbind_any(res, m_restraints);
}
void PhysicsManager::unbind(TriggerInterface* trigger) {
    unbind_any(trigger, m_triggers);
}

}
