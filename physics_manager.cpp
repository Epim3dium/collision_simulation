#include "physics_manager.hpp"
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
    std::vector<ColInfo> col_list;
    for(auto& r1 : m_rigidbodiesQT) {
        auto found = m_rigidbodiesQT.search(r1.item->aabb());
        for(auto& r2 : found) {
            if(r1.item != r2->item && !areIncompatible(*r1.item, *r2->item) && AABBvAABB(r1.item->aabb(), r2->item->aabb()) ) {
                col_list.push_back({r1.item, r2->item});
            }
        }
    }
    return col_list;
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    for(auto& ci : col_list) {
        float restitution = m_selectFrom(ci.r1->material.restitution, ci.r2->material.restitution, bounciness_select);
        float sfriction = m_selectFrom(ci.r1->material.sfriction, ci.r2->material.sfriction, friction_select);
        float dfriction = m_selectFrom(ci.r1->material.dfriction, ci.r2->material.dfriction, friction_select);
        //ewewewewewewwwwww pls don, float delTt judge me
        auto result = m_solver->solve(ci.r1, ci.r2, restitution, sfriction, dfriction);
    }
}
void PhysicsManager::m_processCollisions(float delT) {
    for(auto& r : m_rigidbodiesQT) {
        r.item->collider.now_colliding = nullptr;
    }
    auto col_list = processBroadPhase();
    processNarrowPhase(col_list);
}
static void devideToSegments(std::vector<Rigidbody*> rigidbodies, float segment_size, std::map<int, std::set<Rigidbody*>>& result) {
    vec2f padding(segment_size, segment_size);
    for(auto& r : rigidbodies) {
        auto aabb = r->aabb();
        aabb.min -= padding;
        aabb.max += padding;
        auto hashed = hash(aabb, segment_size);
        for(auto h : hashed)
            result[h].emplace(r);
    }
}
void PhysicsManager::m_updateRestraints(float delT) {
    for(auto& r : m_restraints)
        r->update(delT);
}
void PhysicsManager::m_processDormant(float delT) {
}
void PhysicsManager::m_processTriggers() {
    for(auto& t : m_triggers) {
        for(auto& r : m_rigidbodiesQT) {
            auto man = m_solver->detect(r.item, t);
            if(man.detected) {
                t->onActivation(r.item, man.contact_normal);
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
    for(auto it = m_rigidbodiesQT.begin(); it != m_rigidbodiesQT.end(); it++) {
        if(qlen(it->item->velocity) > 1.f)
            m_rigidbodiesQT.relocate(it, it->item->aabb());
        else
            it->pItem.iterator->first = it->item->aabb();
        m_updateRigidbody(*it->item, delT);
    }
}
void PhysicsManager::update(float delT ) {
    float deltaStep = delT / (float)steps;
    for(int i = 0; i < steps; i++) {
        m_updatePhysics(deltaStep);
        m_updateRestraints(deltaStep);
        m_processCollisions(deltaStep);
    }
    //m_processDormant(delT);
    m_processTriggers();
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
    auto itr = m_rigidbodiesQT.begin();
    for(; itr != m_rigidbodiesQT.end(); itr++) {
        if(itr->item == rb)
            break;
    }
    if(itr != m_rigidbodiesQT.end())
        m_rigidbodiesQT.remove(itr);
}
void PhysicsManager::unbind(RestraintInterface* res) {
    unbind_any(res, m_restraints);
}
void PhysicsManager::unbind(TriggerInterface* trigger) {
    unbind_any(trigger, m_triggers);
}

}
