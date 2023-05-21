#include "physics_manager.hpp"
#include "col_utils.hpp"
#include "collider.hpp"
#include "debug.hpp"
#include "imgui.h"

#include "restraint.hpp"
#include "solver.hpp"
#include "particle_manager.hpp"
#include "rigidbody.hpp"
#include "transform.hpp"

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
    return (!r1.collider->isTrigger || !r2.collider->isTrigger) &&
        !(r1.rigidbody->isDormant() && r2.rigidbody->isDormant()) && 
        (r2.collider->mask.size() == 0 || r1.collider->tag == r2.collider->mask) && 
        (r1.collider->mask.size() == 0 || r2.collider->tag == r1.collider->mask);
}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
    std::vector<PhysicsManager::ColInfo> result;
    std::vector<std::pair<float, RigidManifold>> all;
    for(auto& c : _rigidbodies) {
        auto aabb = c.collider->getAABB(*c.transform);
        all.push_back({aabb.min.x, c});
        all.push_back({aabb.max.x, c});
    }
    std::sort(all.begin(), all.end(),
        [](const std::pair<float, RigidManifold>& p1, const std::pair<float, RigidManifold>& p2) {
            return p1.first < p2.first;
        });
    std::vector<std::pair<RigidManifold, AABB>> open;
    for(auto i : all) {
        auto idx = i.second;
        auto itr = std::find_if(open.begin(), open.end(), 
            [&](const std::pair<RigidManifold, AABB>& p) {
                return p.first == idx;
            });
        if(itr != open.end()) {
            std::swap(*itr, open.back());
            open.pop_back();
            continue;
        }
        auto aabb = i.second.collider->getAABB(*i.second.transform);
        for(auto ii : open) {
            if(isOverlappingAABBAABB(ii.second, aabb))
                result.push_back({idx, ii.first});
        }
        open.push_back({idx, aabb});
    }
    return result;
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    for(auto ci = col_list.begin(); ci != col_list.end(); ci++) {
        if(!areCompatible(ci->first, ci->second))
            continue;
        auto col_info = _solver->detect(ci->first.transform, ci->first.collider, ci->second.transform, ci->second.collider);

        ci->first.collider->notify({*ci->first.collider, *ci->second.collider, col_info});
        col_info.cn *= -1.f;
        ci->second.collider->notify({*ci->second.collider, *ci->first.collider, col_info});
        col_info.cn *= -1.f;

        if(ci->first.collider->isTrigger || ci->second.collider->isTrigger) {
            continue;
        }
        float restitution = selectFrom(ci->first.material->restitution, ci->second.material->restitution, bounciness_select);
        float sfriction = selectFrom(ci->first.material->sfriction, ci->second.material->sfriction, friction_select);
        float dfriction = selectFrom(ci->first.material->dfriction, ci->second.material->dfriction, friction_select);
        _solver->solve(col_info, ci->first, ci->second, restitution, sfriction, dfriction);
    }
}
void PhysicsManager::updateRestraints(float delT) {
    for(auto& r : _restraints)
        r->update(delT);
}
void PhysicsManager::wakeUpAround(const RigidManifold& man) {
    auto area = man.collider->getAABB(*man.transform);
    area.setSize(area.size() * 2.f);
}
#define DORMANT_MIN_VELOCITY 150.f
#define DORMANT_MIN_ANGULAR_VELOCITY 0.1f
void PhysicsManager::updateRigidObj(RigidManifold& man, float delT) {
    if(man.collider->isTrigger)
        return;
    auto& rb = *man.rigidbody;
    //processing dormants
//    if(qlen(rb.velocity) + len(rb.force) * delT < DORMANT_MIN_VELOCITY && abs(rb.angular_velocity) < DORMANT_MIN_ANGULAR_VELOCITY) {
//        rb.time_immobile += delT;
//    }else {
//        //wake up all objects around it
//        if(man.rigidbody->isDormant()) {
//            wakeUpAround(man);
//        }
//        rb.time_immobile = 0.f;
//    }

    if(rb.isDormant()){
        rb.velocity = vec2f();
        rb.angular_velocity = 0.f;
        return;
    }
    if(rb.lockRotation) {
        rb.angular_velocity = 0.f;
        rb.angular_force = 0.f;
    }

    if(!nearlyEqual(qlen(rb.velocity), 0.f))
        rb.velocity -= norm(rb.velocity) * std::clamp(qlen(rb.velocity) * man.material->air_drag, 0.f, len(rb.velocity)) * delT;
    if(!nearlyEqual(rb.angular_velocity, 0.f))
        rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * man.material->air_drag, 0.f, abs(rb.angular_velocity)) * delT;

    rb.velocity += rb.force / rb.mass * delT;
    rb.angular_velocity += rb.angular_force / man.collider->getInertia(rb.mass) * delT;
    man.transform->setPos(man.transform->getPos() + rb.velocity * delT);
    man.transform->setRot(man.transform->getRot() + rb.angular_velocity * delT);
}
void PhysicsManager::updateRigidbodies(float delT) {
    for(auto r : _rigidbodies) {
        updateRigidObj(r, delT);
    }
}
void PhysicsManager::processParticles(ParticleManager& pm) {
}
void PhysicsManager::update(float delT, ParticleManager* pm ) {
    float deltaStep = delT / (float)steps;
    auto col_list = processBroadPhase();
    for(int i = 0; i < steps; i++) {
        updateRestraints(deltaStep);
        updateRigidbodies(deltaStep);
        processNarrowPhase(col_list);
    }

    for(auto r : _rigidbodies) {
        r.rigidbody->force = {0.f, 0.f};
        r.rigidbody->angular_force = 0.f;
    }
    if(pm){
        processParticles(*pm);
    }
}
void PhysicsManager::add(RigidManifold man) {
    _rigidbodies.push_back(man);
}
void PhysicsManager::add(Restraint* restraint) {
    _restraints.push_back(restraint);
}
template<class T>
static void unbind_any(const T obj, std::vector<T>& obj_vec) {
    auto itr = obj_vec.begin();
    for(; itr != obj_vec.end(); itr++) {
        if(*itr == obj)
            break;
    }
    if(itr != obj_vec.end()) {
        obj_vec.erase(itr);
    }
}
void PhysicsManager::remove(RigidManifold rb) {
    unbind_any(rb, _rigidbodies);
}
void PhysicsManager::remove(const Restraint* res) {
    unbind_any<Restraint*>((Restraint*)res, _restraints);
}

}
