#pragma once
#include "game_object_utils.hpp"
#include "solver.hpp"
#include "rigidbody.hpp"
#include "trigger.hpp"
#include "restraint.hpp"
#include "quad_tree.hpp"
#include "game_object.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>
#include <set>


namespace epi {

struct ParticleManager;

/*
 * \brief used to process collision detection and resolution as well as restraints on rigidbodies
 * every Solver, RigidManifold and Trigger have to be bound to be processed, and unbound to stop processing
 * when destroyed all objects will be automaticly unbound
 */
class PhysicsManager : public GameObject, Signal::Observer {
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
    typedef std::pair<RigidManifold, RigidManifold> ColInfo;
    enum class eColType {
        CircCirc,
        PolyPoly,
        CircPoly
    };

    QuadTree<RigidManifold, std::function<AABB(RigidManifold)> > m_rigidbodiesQT;

    std::vector<RigidManifold> m_rigidbodies;
    std::vector<Restraint*> m_restraints;
    std::vector<TriggerInterface*> m_triggers;

    SolverInterface* m_solver = new DefaultSolver();

    std::vector<ColInfo> processBroadPhase();
    void processNarrowPhase(const std::vector<ColInfo>& col_info);

    void m_wakeUpAround(const RigidManifold& man);
    void m_updateRigidObj(RigidManifold& man, float delT);

    void m_updateRigidbodies(float delT);
    void m_updateRestraints(float delT);

    void m_processTriggers();
    void m_processParticles(ParticleManager& pm);
    static AABB getAABBfromRigidbody(RigidManifold man) {
        return man.collider->getAABB();
    }
public:
    //number of physics/collision steps per frame
    size_t steps = 2;
    #define PHYSICS_MANAGER_TYPE (typeid(PhysicsManager).hash_code())
    Property getPropertyList() const override {
        return {PHYSICS_MANAGER_TYPE, "physicsmanager"};
    }
    void onNotify(const GameObject& obj, Signal::Event event) override;
    inline const decltype(m_rigidbodiesQT)& getQuadTree() {
        return m_rigidbodiesQT;
    }

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
    void bind(RigidManifold man);
    //used to add solver that is used to resolve collisions
    inline void bind(SolverInterface* solver) {
        m_solver = solver;
    }
    //used to add restraints applied on rigidbodies bound
    void bind(Restraint* restraint);
    //used to add triggers that will detect rigidbodies bound
    void bind(TriggerInterface* trigger);
    //removes rigidbody from manager
    void unbind(const Rigidbody* rb);
    //removes restraint from manager
    void unbind(const Restraint* restriant);
    //removes trigger from manager
    void unbind(const TriggerInterface* trigger);

    //size should be max simulated size
    PhysicsManager(AABB size) : m_rigidbodiesQT(size, getAABBfromRigidbody) {}
    ~PhysicsManager() {
        notify(*this, Signal::EventDestroyed);
    }
};
}
