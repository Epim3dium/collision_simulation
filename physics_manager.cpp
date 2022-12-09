#include "physics_manager.hpp"

namespace EPI_NAMESPACE {
    enum class eColType {
        CircCirc,
        PolyPoly,
        CircPoly
    };
    struct ColInfo {
        Rigidbody* rb1;
        Rigidbody* rb2;
        float overlap;
        eColType type;
    };
    void PhysicsManager::processCollisions() {
        std::vector<ColInfo> col_list;
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
        if(col_list.size() == 0)
            return;
        std::sort(col_list.begin(), col_list.end(), 
                [](const ColInfo& i1, const ColInfo& i2) {return i1.overlap < i2.overlap; });
        for(auto& ci : col_list) {
            float restitution = selectFrom(ci.rb1->mat.restitution, ci.rb2->mat.restitution, bounciness_select);
            float sfriction = selectFrom(ci.rb1->mat.sfriction, ci.rb2->mat.sfriction, friction_select);
            float dfriction = selectFrom(ci.rb1->mat.dfriction, ci.rb2->mat.dfriction, friction_select);
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
    void PhysicsManager::updatePhysics(float delT) {
        for(auto& p : m_polys) {
            if(p->isStatic)
                continue;
            p->vel.y += grav * delT;
            p->setPos(p->getPos() + p->vel * delT);
            
            if(qlen(p->vel) != 0.f)
                p->vel -= norm(p->vel) * std::clamp(qlen(p->vel) * 0.001f, 0.f, len(p->vel)) * delT;

            p->setRot(p->getRot() + p->ang_vel * delT);
        }
        for(auto& p : m_circs) {
            if(p->isStatic)
                continue;
            p->vel.y += grav * delT;
            p->pos += p->vel * delT;
            
            if(qlen(p->vel) != 0.f)
                p->vel -= norm(p->vel) * std::clamp(qlen(p->vel) * 0.001f, 0.f, len(p->vel)) * delT;
        }
    }
    void PhysicsManager::update() {
        float delT = 1.f / (float)steps;
        for(int i = 0; i < steps; i++) {
            updatePhysics(delT);
            processCollisions();
        }
    }
}
