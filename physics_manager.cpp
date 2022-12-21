#include "physics_manager.hpp"
#include "collision.h"
#include "rigidbody.hpp"
#include <map>
#include <set>
#include <vector>

#define SQR(x) ((x) * (x))

namespace EPI_NAMESPACE {

typedef int hash_vec;

static  hash_vec hash(vec2i pos) {
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

static bool areCompatible(Rigidbody& rb1, Rigidbody& rb2) {
    return (( rb1.isStatic || rb1.collider.isSleeping) && (rb2.isStatic || rb2.collider.isSleeping)) || 
            rb1.collider.layer == rb2.collider.layer;
}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
    std::vector<PhysicsManager::ColInfo> col_list;
    std::map<hash_vec, std::vector<Rigidbody*>> segmented_rigidbodies;
    for(auto& r : m_rigidbodies) {
        r->debugFlag = 0U;
        auto hashed = hash(r->aabb());
        for(auto h : hashed)
            segmented_rigidbodies[h].push_back(r);
    }
    float overlap;
    for(auto& seg_pair : segmented_rigidbodies) {
        auto& vec = seg_pair.second;
        for(int i = 0; i < vec.size(); i++) {
            for(int ii = i + 1; ii < vec.size(); ii++) {
                if(areCompatible(*vec[i], *vec[ii]))
                    continue;
                if( AABBvAABB(vec[i]->aabb(), vec[ii]->aabb()) ) {
                    col_list.push_back({vec[i], vec[ii]});
                    vec[i]->debugFlag = seg_pair.first;
                    vec[ii]->debugFlag = seg_pair.first;
                }
            }
        }
    }
    return col_list;
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    for(auto& ci : col_list) {
        float restitution = m_selectFrom(ci.rb1->mat.restitution, ci.rb2->mat.restitution, bounciness_select);
        float sfriction = m_selectFrom(ci.rb1->mat.sfriction, ci.rb2->mat.sfriction, friction_select);
        float dfriction = m_selectFrom(ci.rb1->mat.dfriction, ci.rb2->mat.dfriction, friction_select);
        //ewewewewewewwwwww pls dont judge me
        auto man = ci.rb1->handleOverlap(ci.rb2);
        if(!man.detected)
            continue;
        processReaction(man, restitution, sfriction, dfriction);
    }
}
void PhysicsManager::m_processCollisions() {
    for(auto& r : m_rigidbodies) {
        r->collider.now_colliding = nullptr;
    }
    std::vector<ColInfo> col_list = processBroadPhase();
    processNarrowPhase(col_list);
}

void PhysicsManager::processDormant() {
}
void PhysicsManager::m_updateRigidbody(Rigidbody& rb, float delT) {
    //updating velocity and physics
    if(rb.isStatic)
        return;
    rb.vel.y += grav * delT;
    if(qlen(rb.vel) > PHYSICS_MANAGER_MIN_VEL_THRESHOLD)
        rb.vel -= norm(rb.vel) * std::clamp(qlen(rb.vel) * rb.mat.air_drag, 0.f, len(rb.vel)) * delT;
    if(rb.ang_vel > PHYSICS_MANAGER_MIN_VEL_THRESHOLD)
        rb.ang_vel -= std::copysign(1.f, rb.ang_vel) * std::clamp(rb.ang_vel * rb.ang_vel * rb.mat.air_drag, 0.f, rb.ang_vel) * delT;

    rb.updateMovement(delT);
}
void PhysicsManager::m_updatePhysics(float delT) {
    for(auto& r : m_rigidbodies)
        m_updateRigidbody(*r, delT);
}
void PhysicsManager::update() {
    float delT = 1.f / (float)steps;
    for(int i = 0; i < steps; i++) {
        m_updatePhysics(delT);
        m_processCollisions();
    }
    processDormant();
}
}
