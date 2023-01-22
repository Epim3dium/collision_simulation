#pragma once
#include "allocator.hpp"
#include "solver.hpp"
#include "rigidbody.hpp"
#include "trigger.hpp"
#include "restraint.hpp"
#include "quad_tree.hpp"

#include <algorithm>
#include <functional>
#include <vector>
#include <set>


namespace EPI_NAMESPACE {

struct ParticleManager;

class PhysicsManager {
public:
    enum class eSelectMode {
        Min,
        Max,
        Avg
    };
private:
    template<class T>
    static T m_selectFrom(T a, T b, eSelectMode mode) {
        switch(mode) {
            case eSelectMode::Avg:
                return (a + b) / 2.f;
            case eSelectMode::Min:
                return std::min(a, b);
            case eSelectMode::Max:
                return std::max(a, b);
        }
    }
    enum class eColType {
        CircCirc,
        PolyPoly,
        CircPoly
    };
    typedef std::pair<Rigidbody*, Rigidbody*> ColInfo;
    std::vector<ColInfo> processBroadPhase();
    void processNarrowPhase(const std::vector<ColInfo>& col_info);
    void processNarrowRange(std::vector<std::pair<Rigidbody*, Rigidbody*>>::const_iterator begining,std::vector<std::pair<Rigidbody*, Rigidbody*>>::const_iterator ending);

    void m_updateRigidbody(Rigidbody& rb, float delT);

    void m_updatePhysics(float delT);
    void m_updateRestraints(float delT);

    void m_processTriggers();
    void m_processParticles(ParticleManager& pm);

    QuadTree<Rigidbody*, std::function<AABB(Rigidbody*)> > m_rigidbodiesQT;
    std::vector<Rigidbody*> m_rigidbodies;

    std::vector<RestraintInterface*> m_restraints;
    std::vector<TriggerInterface*> m_triggers;

    SolverInterface* m_solver = new DefaultSolver();
    static AABB getAABBfromRigidbody(Rigidbody* rb) {
        return rb->getCollider().getAABB();
    }
public:
    float grav = 0.5f;
    size_t steps = 2;
    float min_dormant_velocity = 500.f;
    float min_angular_dormant_velocity = 30.f;
    float segment_size = 100.f;

    void update(float delT, ParticleManager* pm = nullptr);

    eSelectMode bounciness_select = eSelectMode::Min;
    eSelectMode friction_select = eSelectMode::Min;


    inline void bind(Rigidbody* rb) {
        m_rigidbodies.push_back(rb);
    }
    inline void bind(SolverInterface* solver) {
        m_solver = solver;
    }
    inline void bind(RestraintInterface* restraint) {
        m_restraints.push_back(restraint);
    }
    inline void bind(TriggerInterface* trigger) {
        m_triggers.push_back(trigger);
    }
    void unbind(Rigidbody* rb);
    void unbind(RestraintInterface* restriant);
    void unbind(TriggerInterface* trigger);

    PhysicsManager(AABB size) : m_rigidbodiesQT(size, getAABBfromRigidbody) {}
};
}
