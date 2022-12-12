#include "physics_manager.hpp"
#include "collision.h"
#include <vector>

namespace EPI_NAMESPACE {
    void PhysicsManager::processBroadPhase(std::vector<PhysicsManager::ColInfo>& col_list, std::vector<Rigidbody*>& no_col_list) {
        float overlap;
        for(int i = 0; i < m_polys.size(); i++) {
            for(int ii = i + 1; ii < m_polys.size(); ii++) {
                if(possibleIntersection(*m_polys[i], *m_polys[ii]))
                    if(detect(*m_polys[i], *m_polys[ii], nullptr, &overlap)) {
                        col_list.push_back({m_polys[i], m_polys[ii], overlap, eColType::PolyPoly});
                    }
            }
            for(int ii = 0; ii < m_circs.size(); ii++) {
                if(possibleIntersection(*m_circs[ii], *m_polys[i]))
                    if(detect(*m_circs[ii], *m_polys[i], nullptr, &overlap)) {
                        col_list.push_back({m_circs[ii], m_polys[i], overlap, eColType::CircPoly});
                    }
            }
        }
        for(int i = 0; i < m_circs.size(); i++) {
            for(int ii = i + 1; ii < m_circs.size(); ii++) {
                if(detect(*m_circs[i], *m_circs[ii], nullptr, &overlap)) {
                    col_list.push_back({m_circs[i], m_circs[ii], overlap, eColType::CircCirc});
                }
            }
        }
        std::sort(col_list.begin(), col_list.end(), 
                [](const ColInfo& i1, const ColInfo& i2) {return i1.overlap < i2.overlap; });
    }
    void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
        for(auto& ci : col_list) {
            float restitution = m_selectFrom(ci.rb1->mat.restitution, ci.rb2->mat.restitution, bounciness_select);
            float sfriction = m_selectFrom(ci.rb1->mat.sfriction, ci.rb2->mat.sfriction, friction_select);
            float dfriction = m_selectFrom(ci.rb1->mat.dfriction, ci.rb2->mat.dfriction, friction_select);
            switch(ci.type) {
                case eColType::PolyPoly:
                    handle(*(RigidPolygon*)ci.rb1, *(RigidPolygon*)ci.rb2, restitution, sfriction, dfriction);
                    break;
                case eColType::CircPoly:
                    handle(*(RigidCircle*)ci.rb1, *(RigidPolygon*)ci.rb2, restitution, sfriction, dfriction);
                    break;
                case eColType::CircCirc:
                    handle(*(RigidCircle*)ci.rb1, *(RigidCircle*)ci.rb2, restitution, sfriction, dfriction);
                    break;
            }
        }
    }
    void PhysicsManager::m_processCollisions() {
        std::vector<ColInfo> col_list;
        std::vector<Rigidbody*> no_col_list;
        processBroadPhase(col_list, no_col_list);
        processNarrowPhase(col_list);
    }

    void PhysicsManager::m_updateRigidbody(Rigidbody& rb, float delT) {
        if(rb.isStatic || rb.isSleeping)
            return;
        rb.vel.y += grav * delT;
        if(qlen(rb.vel) > PHYSICS_MANAGER_MIN_VEL_THRESHOLD)
            rb.vel -= norm(rb.vel) * std::clamp(qlen(rb.vel) * rb.mat.air_drag, 0.f, len(rb.vel)) * delT;
        rb.updateMovement(delT);
    }
    void PhysicsManager::m_updatePhysics(float delT) {
        for(auto& p : m_polys) {
            m_updateRigidbody(*p, delT);
        }
        for(auto& p : m_circs) {
            m_updateRigidbody(*p, delT);
        }
    }
    void PhysicsManager::update() {
        float delT = 1.f / (float)steps;
        for(int i = 0; i < steps; i++) {
            m_updatePhysics(delT);
            m_processCollisions();
        }
    }
}
