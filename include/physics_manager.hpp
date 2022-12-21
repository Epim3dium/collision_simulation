#include "collision.h"
#include "rigidbody.hpp"
#include <algorithm>
#include <vector>
namespace EPI_NAMESPACE {
enum class eSelectMode {
    Min,
    Max,
    Avg
};
#define SEG_SIZE 300.f
#define PHYSICS_MANAGER_MIN_VEL_THRESHOLD 0.001f
std::vector<int> hash(AABB shape, float seg_size = SEG_SIZE);
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
        Rigidbody* rb1;
        Rigidbody* rb2;
    };

    std::vector<ColInfo> processBroadPhase();
    void processNarrowPhase(const std::vector<ColInfo>& col_info);
    void processDormant();

    void m_processCollisions();

    void m_updateRigidbody(Rigidbody& rb, float delT);
    void m_updatePhysics(float delT);

    std::vector<Rigidbody*> m_rigidbodies;
public:
    float grav = 0.5f;
    size_t steps = 2;
    eSelectMode bounciness_select = eSelectMode::Min;
    eSelectMode friction_select = eSelectMode::Min;

    inline void bind(Rigidbody* rb) {
        m_rigidbodies.push_back(rb);
    }
    void unbind(Rigidbody* rb) {
        auto itr = std::find(m_rigidbodies.begin(), m_rigidbodies.end(), rb);
        if(itr != m_rigidbodies.end())
            m_rigidbodies.erase(itr);
    }
    void update();

    friend RigidPolygon;
};
}
