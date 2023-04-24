#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <memory>
#include <vector>

#include "game_object_utils.hpp"

namespace epi {
/*
* \brief an abstract class for every major component in game
* contains these virtual functions:
*   virtual Property getPropertyList() const, which should return Property containing hashed class type and string name
* it is best to always contain
*  notify(*this, Signal::EventDestroyed)
* in destructor of any class derived from GameObject, since many components use this feature
*/
class GameObject : public Signal::Subject {
public:

    typedef size_t id_type;
    struct Property {
        size_t type_hashed;
        std::string name;
    };
private:
    GameObject* _parent = nullptr;
    GameObjectUtils::ChildContainer _children;
public:
    Tag tag;
    bool isActive = true;
private:
    inline static id_type _generateNextId() { static id_type s_nextID = 1; return s_nextID++; }
    void _copyGameObject(const GameObject& other);
public:
    //returns pointer to parent object
    inline GameObject* getParent() const {return _parent; }
    //returns Properties of this object
    virtual Property getPropertyList() const = 0;

    //adds compenent to this objects subcomponents to be deleted in destructor
    template<class T>
    inline T* addComponent(T* obj) {
        _children.insert(obj); obj->_parent = this;
        return obj;
    }
    //returns component that matches type T, if there is none assert will be failed
    template<class T>
    T& getComponent() {
        return _children.getComponent<T>();
    }
    //performs all safety checks and then casts object to its higher definition

    GameObject& operator=(const GameObject& copy) = delete;
    GameObject(const GameObject& copy) = delete;
    GameObject(const GameObject&& copy) = delete;
    GameObject() { }
    virtual ~GameObject();
};
template<class T>
inline T* safeCast(const GameObject* obj) {
    if(!std::is_base_of<GameObject, T>::value) {
        return nullptr;
    }
    return (T*)obj;
}
template<class T>
T& cast(const GameObject& obj) {
    static_assert(std::is_base_of<GameObject, T>::value, "T must derive from GameObject");
    return (T&)obj;
}

}
#endif
