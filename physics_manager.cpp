#include "physics_manager.hpp"
#include "solver.hpp"
#include "rigidbody.hpp"
#include <cmath>
#include <map>
#include <set>
#include <vector>

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
    std::vector<PhysicsManager::ColInfo> col_list;
    struct BoundInfo {
        Rigidbody* id;
        float val;
        bool isEnding = false;
    };
    std::vector<BoundInfo> infos;
    for(auto& r : m_rigidbodies) {
        infos.push_back({r, r->aabb().min.x});
        infos.push_back({r, r->aabb().max.x, true});
    }
    std::sort(infos.begin(), infos.end(), 
          [](const BoundInfo& bi1, const BoundInfo& bi2) {
              return bi1.val < bi2.val;
          });
    std::map<int, std::vector<Rigidbody*>> open;
    for(auto& i : infos) {
        auto minval = i.id->aabb().min.y;
        auto maxval = i.id->aabb().max.y;
        if(i.isEnding) {
            for(int j = minval / segment_size; j <= maxval / segment_size; j++) {
                if(open.find(j) == open.end())
                    continue;
                auto& vec = open.at(j);
                auto itr = std::find(vec.begin(), vec.end(), i.id);
                if(itr != vec.end())
                    vec.erase(itr);
            }
        } else {
            for(int j = minval / segment_size; j <= maxval / segment_size; j++) {
                auto& vec = open[j];
                for(auto& o : vec) {
                    if(!areIncompatible(*i.id, *o) && AABBvAABB(i.id->aabb(), o->aabb()) ) {
                        col_list.push_back({i.id, o});
                    }
                }
                vec.push_back(i.id);
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
        auto pos1 = ci.r1->aabb().center();
        auto pos2 = ci.r2->aabb().center();
        auto result = m_solver->solve(ci.r1, ci.r2, restitution, sfriction, dfriction);
        if(!result)
            continue;
    }
}
void PhysicsManager::m_processCollisions(float delT) {
    for(auto& r : m_rigidbodies) {
        r->collider.now_colliding = nullptr;
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
    std::map<int, std::set<Rigidbody*> > devided;
    devideToSegments(m_rigidbodies, segment_size, devided);
    for(auto& seg : devided) {
        auto& vec = seg.second;
        bool all_dormant = true;
        for(auto& r : vec) {
            if(r->isStatic)
                continue;
            if(qlen(r->velocity) < MAX_VELOCITY) {
                r->collider.dormant_time += delT;
            } else {
                r->collider.dormant_time = 0;
            }
            if(!r->collider.isDormantCapable())
                all_dormant = false;
        }
        for(auto& r : vec) {
            r->collider.isDormant = all_dormant;
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
    rb.setRot(rb.getRot() + rb.angular_velocity * delT);
}
void PhysicsManager::m_updatePhysics(float delT) {
    for(auto& r : m_rigidbodies)
        m_updateRigidbody(*r, delT);
}
void PhysicsManager::update(float delT ) {
    float deltaStep = delT / (float)steps;
    for(int i = 0; i < steps; i++) {
        m_updatePhysics(deltaStep);
        m_updateRestraints(deltaStep);
        m_processCollisions(deltaStep);
    }
    m_processDormant(delT);
}

}
