#include "physics_manager.hpp"
#include "col_utils.hpp"
#include "collider.hpp"
#include "game_object_utils.hpp"
#include "imgui.h"

#include "restraint.hpp"
#include "solver.hpp"
#include "particle_manager.hpp"
#include "rigidbody.hpp"
#include "transform.hpp"
#include "trigger.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <map>
#include <mutex>
#include <memory>
#include <set>
#include <stdexcept>
#include <sys/_types/_size_t.h>
#include <vector>
#include <thread>
#include <mutex>


namespace epi {

static bool areCompatible(RigidManifold r1, RigidManifold r2) {
    return !(r1.rigidbody->isDormant() && r2.rigidbody->isDormant()) && 
        (r2.collider->mask.size() == 0 || r1.collider->tag == r2.collider->mask) && 
        (r1.collider->mask.size() == 0 || r2.collider->tag == r1.collider->mask);
}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
    return m_rigidbodiesQT.findAllIntersections();
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    for(auto ci = col_list.begin(); ci != col_list.end(); ci++) {
        if(!areCompatible(ci->first, ci->second))
            continue;
        float restitution = m_selectFrom(ci->first.material->restitution, ci->second.material->restitution, bounciness_select);
        float sfriction = m_selectFrom(ci->first.material->sfriction, ci->second.material->sfriction, friction_select);
        float dfriction = m_selectFrom(ci->first.material->dfriction, ci->second.material->dfriction, friction_select);
        //ewewewewewewwwwww pls don, float delTt judge me
        auto result = m_solver->solve(ci->first, ci->second, restitution, sfriction, dfriction);
    }
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
            auto man = m_solver->detect(r.collider, &t->getCollider());
            if(man.detected) {
                t->onActivation(r.rigidbody, man.cn);
            }
        }
    }
}

void PhysicsManager::m_wakeUpAround(const RigidManifold& man) {
    auto area = man.collider->getAABB();
    area.setSize(area.size() * 2.f);
    for(auto& r : m_rigidbodiesQT.query(area))
        r.rigidbody->time_immobile = 0.f;
}
#define DORMANT_MIN_VELOCITY 150.f
#define DORMANT_MIN_ANGULAR_VELOCITY 0.1f
void PhysicsManager::m_updateRigidObj(RigidManifold& man, float delT) {
    auto& rb = *man.rigidbody;
    if(rb.inertia == -1.f)
        rb.inertia = man.collider->calcInertia(rb.mass);
    //processing dormants
    if(qlen(rb.velocity) + len(rb.force) * delT < DORMANT_MIN_VELOCITY && abs(rb.angular_velocity) < DORMANT_MIN_ANGULAR_VELOCITY) {
        rb.time_immobile += delT;
    }else {
        //wake up all objects around it
        if(man.rigidbody->isDormant()) {
            m_wakeUpAround(man);
        }
        rb.time_immobile = 0.f;
    }

    if(rb.isDormant()){
        rb.velocity = vec2f();
        return;
    }

    if(rb.isStatic)
        return;
    if(qlen(rb.velocity) > 0.001f)
        rb.velocity -= norm(rb.velocity) * std::clamp(qlen(rb.velocity) * man.material->air_drag, 0.f, len(rb.velocity)) * delT;
    if(abs(rb.angular_velocity) > 0.001f)
        rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * man.material->air_drag, 0.f, abs(rb.angular_velocity)) * delT;

    rb.velocity += rb.force / rb.mass * delT;
    rb.angular_velocity += rb.angular_force / rb.inertia * delT;
    auto p = man.transform->getPos();
    man.transform->setPos(p + rb.velocity * delT);

    if(!rb.lockRotation)
        man.transform->setRot(man.transform->getRot() + rb.angular_velocity * delT);
}
void PhysicsManager::m_updateRigidbodies(float delT) {
    for(auto r : m_rigidbodies) {
        m_updateRigidObj(r, delT);
        m_rigidbodiesQT.update(r);
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
            switch(o.collider->getType()) {
                case eCollisionShape::Circle:
                    if(isOverlappingPointCircle(p.pos, ((CircleCollider*)o.collider)->getShape())) {
                        p.isActive = false;
                    }
                break;
                case eCollisionShape::Polygon:
                    if(isOverlappingPointPoly(p.pos, ((PolygonCollider*)o.collider)->getShape()))
                        p.isActive = false;
                break;
                default:
                    throw std::invalid_argument("not implemeted yet");
                break;
            }
        }
    }
}
void PhysicsManager::update(float delT, ParticleManager* pm ) {
    float deltaStep = delT / (float)steps;
    //adding only once per frame since most probably if 2 aabbs dont overlap at the start of the frame they will not overlap at the end and if they do that will be dealt of in the next frame

    for(int i = 0; i < steps; i++) {
        m_updateRestraints(deltaStep);
        m_updateRigidbodies(deltaStep);

        auto col_list = processBroadPhase();

        processNarrowPhase(col_list);
    }

    for(auto r : m_rigidbodies) {
        r.rigidbody->force = {0.f, 0.f};
        r.rigidbody->angular_force = 0.f;
    }
    m_processTriggers();
    if(pm){
        m_processParticles(*pm);
    }
}
void PhysicsManager::onNotify(const GameObject& obj, Signal::Event event)  {
    if(event != Signal::EventDestroyed)
        return;
    if(obj.getPropertyList().type_hashed == RIGIDBODY_TYPE) {
        unbind(safeCast<Rigidbody>(&obj));
    }else if(obj.getPropertyList().type_hashed == TRIGGER_TYPE) {
        unbind(safeCast<TriggerInterface>(&obj));
    }else if(obj.getPropertyList().type_hashed == RESTRAINT_TYPE) {
        unbind(safeCast<Restraint>(&obj));
    }
}
void PhysicsManager::bind(RigidManifold man) {
    m_rigidbodies.push_back(man);
    man.rigidbody->addObserver(this);
}
void PhysicsManager::bind(Restraint* restraint) {
    m_restraints.push_back(restraint);
    restraint->addObserver(this);
}
void PhysicsManager::bind(TriggerInterface* trigger) {
    m_triggers.push_back(trigger);
    trigger->addObserver(this);
}
void PhysicsManager::unbind(const Rigidbody* rb) {
    auto itr = m_rigidbodies.begin();
    for(; itr != m_rigidbodies.end(); itr++) {
        if(itr->rigidbody == rb) {
            m_wakeUpAround(*itr);
            m_rigidbodiesQT.remove(*itr);
            m_rigidbodies.erase(itr);
            break;
        }
    }
}
template<class T>
static void unbind_any(const T* obj, std::vector<T*>& obj_vec) {
    auto itr = obj_vec.begin();
    for(; itr != obj_vec.end(); itr++) {
        if(*itr == obj)
            break;
    }
    if(itr != obj_vec.end()) {
        obj_vec.erase(itr);
    }
}
void PhysicsManager::unbind(const Restraint* res) {
    unbind_any(res, m_restraints);
}
void PhysicsManager::unbind(const TriggerInterface* trigger) {
    unbind_any(trigger, m_triggers);
}

}
