#pragma once
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/System/Vector2.hpp"
#include "game_object.hpp"
#include "game_object_utils.hpp"

namespace epi {

typedef sf::Vector2f vec2f;

namespace Signal {
    namespace Transform {
        static constexpr const char* EventPosChange = "transformposchange";
        static constexpr const char* EventRotChange = "transformrotchange";
        static constexpr const char* EventScaleChange = "transformscalechange";
    }
}
class Transform : public GameObject {
    vec2f _pos;
    vec2f _scale;
    float _rot;
public:
    #define TRANSFORM_TYPE (typeid(Transform).hash_code())
    Property getPropertyList() const override {
        return {TRANSFORM_TYPE, "transform"};
    }
    vec2f getPos() const  {
        return this->_pos;
    }
    void setPos(vec2f v) {
        this->_pos = v;
        notify(*this, Signal::Transform::EventPosChange);
    }

    vec2f getScale() const {
        return _scale;
    }
    void setScale(vec2f v) {
        _scale = v;
        notify(*this, Signal::Transform::EventScaleChange);
    }

    float getRot() const {
        return _rot;
    }
    void setRot(float r) {
        _rot = r;
        notify(*this, Signal::Transform::EventRotChange);
    }
    Transform() : _scale(1.f, 1.f), _rot(0.f), _pos(0, 0) {
    }
    ~Transform() {
        this->notify(*this, Signal::EventDestroyed);
    }
};

}
