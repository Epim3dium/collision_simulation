#include "restraint.hpp"
#include "solver.hpp"
#include "rigidbody.hpp"
#include <algorithm>
#include <functional>
#include <vector>
namespace EPI_NAMESPACE {
enum class eSelectMode {
    Min,
    Max,
    Avg
};
std::vector<int> hash(AABB shape, float seg_size);
class PhysicsManager {
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
    struct ColInfo {
        Rigidbody* r1;
        Rigidbody* r2;
    };
    std::vector<ColInfo> processBroadPhase();
    void processNarrowPhase(const std::vector<ColInfo>& col_info);

    void m_processCollisions(float delT);

    void m_updateRigidbody(Rigidbody& rb, float delT);
    void m_updatePhysics(float delT);
    void m_updateRestraints(float delT);
    void m_processDormant(float delT);

    std::vector<Rigidbody*> m_rigidbodies;
    std::vector<RestraintInterface*> m_restraints;
    InterfaceSolver* m_solver = new DefaultSolver();
public:
    float grav = 0.5f;
    size_t steps = 2;
    float min_dormant_velocity = 500.f;
    float min_angular_dormant_velocity = 30.f;
    float segment_size = 300.f;

    eSelectMode bounciness_select = eSelectMode::Min;
    eSelectMode friction_select = eSelectMode::Min;

    inline void bind(Rigidbody* rb) {
        m_rigidbodies.push_back(rb);
    }
    inline void bind(InterfaceSolver* solver) {
        m_solver = solver;
    }
    inline void bind(RestraintInterface* restraint) {
        m_restraints.push_back(restraint);
    }
    void unbind(Rigidbody* rb) {
        auto itr = m_rigidbodies.begin();
        for(; itr != m_rigidbodies.end(); itr++) {
            if(*itr == rb)
                break;
        }
        if(itr != m_rigidbodies.end())
            m_rigidbodies.erase(itr);
    }
    void unbind(RestraintInterface* rb) {
        auto itr = m_restraints.begin();
        for(; itr != m_restraints.end(); itr++) {
            if(*itr == rb)
                break;
        }
        if(itr != m_restraints.end())
            m_restraints.erase(itr);
    }
    void update(float delT);

    friend RigidPolygon;
};
}
