#include "game_object.hpp"
#include "game_object_utils.hpp"
#include <algorithm>
#include <memory>
namespace epi {

void GameObject::_copyGameObject(const GameObject& other) {
    _parent = other._parent;
    _children = other._children;
    isActive = other.isActive;
    tag = other.tag;
}
GameObject::~GameObject() {
}

}
