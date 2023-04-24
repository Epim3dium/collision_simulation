
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Window/VideoMode.hpp"
#include "SFML/Window/Window.hpp"
#include "SFML/Window.hpp"

#include "game_object.hpp"
#include "game_object_utils.hpp"
#include "types.hpp"

namespace epi {
class Camera : public GameObject {
public:
    vec2f position;
    float smoothing;
    Camera() {
    }
};
}
