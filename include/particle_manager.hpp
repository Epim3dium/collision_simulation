#include "particle.hpp"
#include "types.hpp"
#include <cstddef>
#include <sys/_types/_size_t.h>
#include <vector>

namespace EPI_NAMESPACE {
struct PhysicsManager;
class ParticleManager {
    std::vector<Particle> m_particles;
    size_t m_cur_idx;
    size_t m_active_particles;

    size_t findNextInactive();
public:
    void emit(size_t count, const std::vector<Particle::InitInerface*>& inits) {
        for(size_t i = 0; i < count; i++) {
            size_t idx = findNextInactive();
            if(idx == -1)
                return;
            m_particles[idx].init(inits);
        }
        m_active_particles += count;
    }
    void emit(size_t count, const Particle::InitList& inits) {
        emit(count, inits.inits);
    }
    void update(float delT);
    void draw(Window& rw);


    ParticleManager(size_t max_particle_count) : m_particles(max_particle_count), m_cur_idx(0U), m_active_particles(0U) {}
    friend PhysicsManager;
};
}
