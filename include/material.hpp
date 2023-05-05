
#include "types.hpp"

namespace epi {
// basic class to store physical material properties
struct Material : public GameObject {
    #define MATERIAL_TYPE (typeid(Material).hash_code())
    Property getPropertyList() const override {
        return {MATERIAL_TYPE, "material"};
    }
    float restitution = 0.0f;
    float sfriction = 0.4f;
    float dfriction = 0.4f;
    float air_drag = 0.001f;
};
}
