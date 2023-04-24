#include "particle_manager.hpp"
#include <cstddef>

namespace epi {
size_t ParticleManager::findNextInactive() {
    size_t total_searches = 0;
    while(total_searches < m_particles.size()) {
        if(!m_particles[m_cur_idx].isActive)
            return m_cur_idx;
        m_cur_idx = (m_cur_idx + 1) % m_particles.size();
        total_searches++;
    }
    return -1;
}
void ParticleManager::update(float delT) {
    size_t count = 0U;
    for(auto pitr = m_particles.begin(); pitr != m_particles.end() && count < m_active_particles; pitr++) {
        if(pitr->isActive) {
            pitr->update(delT);
            count++;
        }
    }
    m_active_particles = count;
}
void ParticleManager::draw(Window& rw) {
    for(auto& p : m_particles)
        if(p.isActive)
            p.draw(rw);
}


}
