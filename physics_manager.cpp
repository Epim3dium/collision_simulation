#include "physics_manager.hpp"
#include "collision.h"
#include "rigidbody.hpp"
#include <cmath>
#include <map>
#include <set>
#include <vector>

#define SQR(x) ((x) * (x))

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

static bool areCompatible(Rigidbody& rb1, Rigidbody& rb2) {
    return (( rb1.isStatic && rb2.isStatic) || 
            rb1.collider.layer == rb2.collider.layer);
}
std::map<int, std::vector<PhysicsManager::RigidObj>> PhysicsManager::devideToSegments() {
    std::map<hash_vec, std::vector<RigidObj>> segmented_rigidbodies;
    for(auto& r : m_rigidbodies) {
        r.rb->debugFlag = 0U;
        auto hashed = hash(r.rb->aabb(), segment_size);
        for(auto h : hashed)
            segmented_rigidbodies[h].push_back(r);
    }
    return segmented_rigidbodies;

}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase(const std::map<int, std::vector<PhysicsManager::RigidObj>>& segmented_rigidbodies) {
    std::vector<PhysicsManager::ColInfo> col_list;
    float overlap;
    for(auto& seg_pair : segmented_rigidbodies) {
        auto& vec = seg_pair.second;
        for(int i = 0; i < vec.size(); i++) {
            for(int ii = i + 1; ii < vec.size(); ii++) {
                if(areCompatible(*vec[i].rb, *vec[ii].rb))
                    continue;
                if( AABBvAABB(vec[i].rb->aabb(), vec[ii].rb->aabb()) ) {
                    col_list.push_back({vec[i], vec[ii]});
                }
            }
        }
    }
    return col_list;
}
void PhysicsManager::processDormants(const std::map<int, std::vector<PhysicsManager::RigidObj>>& segmented_rigidbodies, float delT) {
    for(auto& seg_pair : segmented_rigidbodies) {
        bool all_dormant = true;
        for(auto& r : seg_pair.second) {
            if(qlen(r.rb->velocity) > min_dormant_velocity || r.rb->angular_velocity > min_angular_dormant_velocity) {
                all_dormant = false;
                break;
            }
        }
        for(auto& r : seg_pair.second) {
            r.rb->collider.dormant_time = (all_dormant ? r.rb->collider.dormant_time + delT: 0);
        }
    }
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    for(auto& ci : col_list) {
        float restitution = m_selectFrom(ci.r1.rb->material.restitution, ci.r2.rb->material.restitution, bounciness_select);
        float sfriction = m_selectFrom(ci.r1.rb->material.sfriction, ci.r2.rb->material.sfriction, friction_select);
        float dfriction = m_selectFrom(ci.r1.rb->material.dfriction, ci.r2.rb->material.dfriction, friction_select);
        //ewewewewewewwwwww pls don, float delTt judge me
        auto man = ci.r1.rb->handleOverlap(ci.r2.rb);
        if(!man.detected)
            continue;
        processReaction(man, restitution, sfriction, dfriction);
        man.r1->collider.now_colliding = man.r2;
        man.r2->collider.now_colliding = man.r1;
        if(ci.r1.onHit)
            ci.r1.onHit(ci.r1.rb, ci.r2.rb);
        if(ci.r2.onHit)
            ci.r2.onHit(ci.r2.rb, ci.r1.rb);
    }
}
void PhysicsManager::m_processCollisions(float delT) {
    for(auto& r : m_rigidbodies) {
        r.rb->collider.now_colliding = nullptr;
    }
    auto segments = devideToSegments();
    auto col_list = processBroadPhase(segments);
    processDormants(segments, delT);
    processNarrowPhase(col_list);
}

void PhysicsManager::m_updateRigidbody(Rigidbody& rb, float delT) {
    //updating velocity and physics
    if(rb.isStatic)
        return;
    if(rb.collider.isDormant())
        return;
    rb.velocity.y += grav * delT;
    if(qlen(rb.velocity) > 0.001f)
        rb.velocity -= norm(rb.velocity) * std::clamp(qlen(rb.velocity) * rb.material.air_drag, 0.f, len(rb.velocity)) * delT;
    rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * rb.material.air_drag, 0.f, rb.angular_velocity) * delT;

    rb.updateMovement(delT);
}
void PhysicsManager::m_updatePhysics(float delT) {
    for(auto& r : m_rigidbodies)
        m_updateRigidbody(*r.rb, delT);
}
void PhysicsManager::update(float delT ) {
    float deltaStep = delT / (float)steps;
    for(int i = 0; i < steps; i++) {
        m_updatePhysics(deltaStep);
        m_processCollisions(deltaStep);
    }
}
}
