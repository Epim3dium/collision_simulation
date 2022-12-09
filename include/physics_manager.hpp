#include "collision.h"
namespace EPI_NAMESPACE {
    enum class eSelectMode {
        Min,
        Max,
        Avg
    };
    class PhysicsManager {
        template<class T>
        static T selectFrom(T a, T b, eSelectMode mode) {
            switch(mode) {
                case eSelectMode::Avg:
                    return (a + b) / 2.f;
                case eSelectMode::Min:
                    return std::min(a, b);
                case eSelectMode::Max:
                    return std::max(a, b);
            }
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
        }
        inline void bind(RigidCircle* rb) {
            m_circs.push_back(rb);
        }
        void unbind(RigidPolygon* rb) {
            auto itr = std::find(m_polys.begin(), m_polys.end(), rb);
            if(itr != m_polys.end())
                m_polys.erase(itr);
        }
        void unbind(RigidCircle* rb) {
            auto itr = std::find(m_circs.begin(), m_circs.end(), rb);
            if(itr != m_circs.end())
                m_circs.erase(itr);
        }
        void processCollisions();
        void updatePhysics(float delT);
        void update();

        friend RigidPolygon;
    };
}
