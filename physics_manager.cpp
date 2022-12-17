#include "physics_manager.hpp"
#include "collision.h"
#include <vector>

#define SQR(x) ((x) * (x))
namespace EPI_NAMESPACE {
    static bool areCompatible(Rigidbody& rb1, Rigidbody& rb2) {
        return (( rb1.isStatic || rb1.collider.isSleeping) && (rb2.isStatic || rb2.collider.isSleeping)) || 
                rb1.collider.layer == rb2.collider.layer;
    }
    std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
        std::vector<PhysicsManager::ColInfo> col_list;
        float overlap;
        for(int i = 0; i < m_polys.size(); i++) {
            for(int ii = i + 1; ii < m_polys.size(); ii++) {
                if(areCompatible(*m_polys[i], *m_polys[ii]))
                    continue;
                if(possibleIntersection(*m_polys[i], *m_polys[ii]))
                    col_list.push_back({m_polys[i], m_polys[ii], overlap, eColType::PolyPoly, (m_polys[i]->getPos() + m_polys[ii]->getPos()) / 2.f});
            }
            for(int ii = 0; ii < m_circs.size(); ii++) {
                if(areCompatible(*m_polys[i], *m_circs[ii]))
                    continue;
                if(possibleIntersection(*m_circs[ii], *m_polys[i]))
                    col_list.push_back({m_circs[ii], m_polys[i], overlap, eColType::CircPoly, (m_polys[i]->getPos() + m_circs[ii]->pos) / 2.f});
            }
        }
        for(int i = 0; i < m_circs.size(); i++) {
            for(int ii = i + 1; ii < m_circs.size(); ii++) {
                if(areCompatible(*m_circs[i], *m_circs[ii]))
                    continue;
                if(qlen(m_circs[i]->pos - m_circs[ii]->pos) < SQR(m_circs[i]->radius + m_circs[ii]->radius)) {
                    col_list.push_back({m_circs[i], m_circs[ii], overlap, eColType::CircCirc, (m_circs[i]->pos + m_circs[ii]->pos) / 2.f});
                }
            }
        }
        return col_list;
    }
#define SORT_DIF_OVERLAP 1.f
    void PhysicsManager::sortCollisionList(std::vector<PhysicsManager::ColInfo>& col_list) {
        std::sort(col_list.begin(), col_list.end(), 
            [](const ColInfo& i1, const ColInfo& i2) {
                if(abs(i1.overlap - i2.overlap) < SORT_DIF_OVERLAP)
                    return i1.col_pos.y > i2.col_pos.y; 
                return i1.overlap < i2.overlap;
            });
    }
    void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
        for(auto& ci : col_list) {
            float restitution = m_selectFrom(ci.rb1->mat.restitution, ci.rb2->mat.restitution, bounciness_select);
            float sfriction = m_selectFrom(ci.rb1->mat.sfriction, ci.rb2->mat.sfriction, friction_select);
            float dfriction = m_selectFrom(ci.rb1->mat.dfriction, ci.rb2->mat.dfriction, friction_select);
            bool result;
            //ewewewewewewwwwww pls dont judge me
            switch(ci.type) {
                case eColType::PolyPoly: {
                    result = handle(*(RigidPolygon*)ci.rb1, *(RigidPolygon*)ci.rb2, restitution, sfriction, dfriction);
                 } break;
                case eColType::CircPoly: {
                    result = handle(*(RigidCircle*)ci.rb1, *(RigidPolygon*)ci.rb2, restitution, sfriction, dfriction);
                 } break;
                case eColType::CircCirc: {
                    result = handle(*(RigidCircle*)ci.rb1, *(RigidCircle*)ci.rb2, restitution, sfriction, dfriction);
                 } break;
            }
            if(result) {
                ci.rb1->collider.now_colliding = ci.rb2;
                ci.rb2->collider.now_colliding = ci.rb1;
            }
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
