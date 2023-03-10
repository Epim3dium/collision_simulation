#include "physics_manager.hpp"
#include "col_utils.hpp"
#include "imgui.h"

#include "solver.hpp"
#include "particle_manager.hpp"
#include "rigidbody.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <map>
#include <mutex>
#include <memory>
#include <set>
#include <sys/_types/_size_t.h>
#include <vector>
#include <thread>
#include <mutex>


namespace EPI_NAMESPACE {

template<class T>
static bool hasDuplicates(const std::set<T>& s1, const std::set<T>& s2) {
    for(auto i : s2) {
        if(s1.contains(i))
            return true;
    }
    return false;
}
static bool areCompatible(Rigidbody& rb1, Rigidbody& rb2) {
    return !(rb1.isDormant() && rb2.isDormant()) && 
        (rb2.collision_mask.size() == 0 || hasDuplicates(rb1.collision_layer, rb2.collision_mask)) && 
        (rb1.collision_mask.size() == 0 || hasDuplicates(rb2.collision_layer, rb1.collision_mask));
}
template <class T>
static void checkAndDeleteOrphansFromVec(std::vector<std::shared_ptr<T> >& vec) {
    for(int i = 0; i < vec.size(); i++) {
        if(vec[i].unique()) {
            vec.erase(vec.begin() + i);
            i--;
        }
    }
}
void PhysicsManager::m_checkAndDeleteOrphans() {
    checkAndDeleteOrphansFromVec(m_rigidbodies);
    checkAndDeleteOrphansFromVec(m_triggers);
    checkAndDeleteOrphansFromVec(m_restraints);
}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
    return m_rigidbodiesQT.findAllIntersections();
}
void PhysicsManager::processNarrowRange(std::vector<std::pair<Rigidbody*, Rigidbody*>>::const_iterator begining,std::vector<std::pair<Rigidbody*, Rigidbody*>>::const_iterator ending) {
    for(auto ci = begining; ci != ending; ci++) {
        if(!areCompatible(*ci->first, *ci->second))
            continue;
        float restitution = m_selectFrom(ci->first->material.restitution, ci->second->material.restitution, bounciness_select);
        float sfriction = m_selectFrom(ci->first->material.sfriction, ci->second->material.sfriction, friction_select);
        float dfriction = m_selectFrom(ci->first->material.dfriction, ci->second->material.dfriction, friction_select);
        //ewewewewewewwwwww pls don, float delTt judge me
        auto result = m_solver->solve(ci->first, ci->second, restitution, sfriction, dfriction);
        if(result.detected) {
            ci->first->pressure += result.overlap;
            ci->second->pressure += result.overlap;
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
        if(t->getCollider().getAABB().min == t->getCollider().getAABB().max)
            continue;
        auto possible = m_rigidbodiesQT.query(t->getCollider().getAABB());
        for(auto& r : possible) {
            auto man = m_solver->detect(&t->getCollider(), &r->getCollider(), nullptr, nullptr);
            if(man.detected) {
                t->onActivation(r, man.cn);
            }
        }
    }
}

#define DORMANT_MIN_VELOCITY 150.f
#define DORMANT_MIN_ANGULAR_VELOCITY 0.1f
void PhysicsManager::m_updateRigidbody(Rigidbody& rb, float delT) {
    if(rb.isStatic)
        return;
    //updating velocity and physics
    if(qlen(rb.velocity) < DORMANT_MIN_VELOCITY && abs(rb.angular_velocity) < DORMANT_MIN_ANGULAR_VELOCITY) {
        rb.time_immobile += delT;
    }else {
        //wake up all objects around it
        if(rb.isDormant()) {
            auto area = rb.getCollider().getAABB();
            area.setSize(area.size() * 2.f);
            for(auto& r : m_rigidbodiesQT.query(area))
                r->time_immobile = 0.f;
        }
        rb.time_immobile = 0.f;
    }
    if(rb.isDormant()){
        rb.velocity = vec2f();
        return;
    }
    //applying drag
    if(qlen(rb.velocity) > 0.001f)
        rb.velocity -= norm(rb.velocity) * std::clamp(qlen(rb.velocity) * rb.material.air_drag, 0.f, len(rb.velocity)) * delT;
    if(abs(rb.angular_velocity) > 0.001f)
        rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * rb.material.air_drag, 0.f, abs(rb.angular_velocity)) * delT;

    rb.getCollider().setPos(rb.getCollider().getPos() + rb.velocity * delT);

    if(!rb.lockRotation)
        rb.getCollider().setRot(rb.getCollider().getRot() + rb.angular_velocity * delT);
}
void PhysicsManager::m_updatePhysics(float delT) {
    for(auto r : m_rigidbodies) {
        m_updateRigidbody(*r, delT);
    }
}
void PhysicsManager::m_processParticles(ParticleManager& pm) {
    if(pm.m_active_particles == 0)
        return;
    for(auto& p : pm.m_particles) {
        if(!p.isActive)
            continue; 
        auto open = m_rigidbodiesQT.query({p.pos - vec2f(0.1f, 0.1f), p.pos + vec2f(0.1f, 0.1f)});
        for(auto& o : open) {
            switch(o->getCollider().getType()) {
                case eCollisionShape::Circle:
                    if(isOverlappingPointCircle(p.pos, ((RigidCircle*)o)->collider.getShape())) {
                        p.isActive = false;
                    }
                break;
                case eCollisionShape::Polygon:
                    if(isOverlappingPointPoly(p.pos, ((RigidPolygon*)o)->collider.getShape()))
                        p.isActive = false;
                break;
            }
        }
    }
}
void PhysicsManager::update(float delT, ParticleManager* pm ) {
    float deltaStep = delT / (float)steps;
    for(auto r : m_rigidbodies) {
        r->pressure = 0.f;
    }
    //adding only once per frame since most probably if 2 aabbs dont overlap at the start of the frame they will not overlap at the end and if they do that will be dealt of in the next frame
    m_rigidbodiesQT.clear();
    m_checkAndDeleteOrphans();
    m_rigidbodiesQT.updateLeafes();
    for(auto r : m_rigidbodies)
        m_rigidbodiesQT.add(r.get());

    for(int i = 0; i < steps; i++) {
        m_updatePhysics(deltaStep);
        auto col_list = processBroadPhase();
        m_updateRestraints(deltaStep);

        processNarrowPhase(col_list);

    }
    m_processTriggers();
    if(pm){
        m_processParticles(*pm);
    }
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
void PhysicsManager::unbind(std::shared_ptr<Rigidbody> rb) {
    unbind_any(rb, m_rigidbodies);
}
void PhysicsManager::unbind(std::shared_ptr<RestraintInterface> res) {
    unbind_any(res, m_restraints);
}
void PhysicsManager::unbind(std::shared_ptr<TriggerInterface> trigger) {
    unbind_any(trigger, m_triggers);
}

}
