#include "collision.h"
#include <algorithm>
#include <vector>
namespace EPI_NAMESPACE {
    enum class eSelectMode {
        Min,
        Max,
        Avg
    };
#define PHYSICS_MANAGER_MIN_VEL_THRESHOLD 0.001f
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
            float overlap;
            eColType type;
            vec2f col_pos;
        };

        std::vector<ColInfo> processBroadPhase();
        void sortCollisionList(std::vector<ColInfo>& col_list);
        void processNarrowPhase(const std::vector<ColInfo>& col_info);
        void processDormant();

        void m_processCollisions();

        void m_updateRigidbody(Rigidbody& rb, float delT);
        void m_updatePhysics(float delT);

        std::vector<Rigidbody*> m_rigidbodies;
        void m_unbindFromRb(Rigidbody* rb) {
            auto itr = std::find(m_rigidbodies.begin(), m_rigidbodies.end(), rb);
            if(itr != m_rigidbodies.end())
                m_rigidbodies.erase(itr);
        }
    public:
        std::vector<RigidPolygon*> m_polys;
        std::vector<RigidCircle*> m_circs;


        float grav = 0.5f;
        size_t steps = 2;
        eSelectMode bounciness_select = eSelectMode::Min;
        eSelectMode friction_select = eSelectMode::Min;

        inline void bind(RigidPolygon* rb) {
            m_polys.push_back(rb);
            m_rigidbodies.push_back(rb);
        }
        inline void bind(RigidCircle* rb) {
            m_circs.push_back(rb);
            m_rigidbodies.push_back(rb);
        }
        void unbind(RigidPolygon* rb) {
            m_unbindFromRb(rb);
            auto itr = std::find(m_polys.begin(), m_polys.end(), rb);
            if(itr != m_polys.end())
                m_polys.erase(itr);
        }
        void unbind(RigidCircle* rb) {
            m_unbindFromRb(rb);
            auto itr = std::find(m_circs.begin(), m_circs.end(), rb);
            if(itr != m_circs.end())
                m_circs.erase(itr);
        }
        void update();

        friend RigidPolygon;
    };
}
