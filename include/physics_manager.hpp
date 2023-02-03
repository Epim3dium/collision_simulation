#pragma once
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
    //number of physics/collision steps per frame
    size_t steps = 2;

    /*
    * updates all rigidbodies bound applying their velocities and resoving collisions
    * @param pm is particle manager, if bound collisions for all active particles will be resolved(colliding particles will become inactive)
    */
    void update(float delT, ParticleManager* pm = nullptr);

    //mode used to select bounce when colliding
    eSelectMode bounciness_select = eSelectMode::Min;
    //mode used to select friciton when colliding
    eSelectMode friction_select = eSelectMode::Min;


    //used to add any rigidbody
    inline void bind(Rigidbody* rb) {
        m_rigidbodies.push_back(rb);
    }
    //used to add solver that is used to resolve collisions
    inline void bind(SolverInterface* solver) {
        m_solver = solver;
    }
    //used to add restraints applied on rigidbodies bound
    inline void bind(RestraintInterface* restraint) {
        m_restraints.push_back(restraint);
    }
    //used to add triggers that will detect rigidbodies bound
    inline void bind(TriggerInterface* trigger) {
        m_triggers.push_back(trigger);
    }
    //removes rigidbody from manager
    void unbind(Rigidbody* rb);
    //removes restraint from manager
    void unbind(RestraintInterface* restriant);
    //removes trigger from manager
    void unbind(TriggerInterface* trigger);

    //size should be max simulated size
    PhysicsManager(AABB size) : m_rigidbodiesQT(size, getAABBfromRigidbody) {}
};
}
