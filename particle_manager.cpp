#include "particle_manager.hpp"
#include <cstddef>

namespace EPI_NAMESPACE {
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
    for(auto& p : m_particles)
        if(p.isActive)
            p.update(delT);

}
void ParticleManager::draw(Window& rw) {
    for(auto& p : m_particles)
        if(p.isActive)
            p.draw(rw);
}


}
