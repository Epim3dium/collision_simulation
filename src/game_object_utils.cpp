#include "game_object_utils.hpp"
#include "game_object.hpp"

using epi::GameObjectUtils::ChildContainer;

void ChildContainer::insert(GameObject* obj) {
    _array_pointers.push_back(obj);
    _typed_members[obj->getPropertyList().type_hashed] = obj;
}
void ChildContainer::erase(GameObject* obj) {
    {
        auto itr = std::find(_array_pointers.begin(), _array_pointers.end(), obj);
        if(itr != _array_pointers.end())
            _array_pointers.erase(itr);
        else
            return;
    }
    {
        auto itr = _typed_members.find(obj->getPropertyList().type_hashed);
        if(itr != _typed_members.end() && itr->second == obj) {
            _typed_members.erase(itr);
        }
    }
}
ChildContainer::ChildContainer() {
    _array_pointers.reserve(16U);
}


using epi::Signal::Observer;
using epi::Signal::Subject;
void Observer::_removeSubject(const Subject* s) {
    auto itr =std::find(_subjects_observed.begin(), _subjects_observed.end(), s);
    if(itr != _subjects_observed.end()) {
        _subjects_observed.erase(itr);
    }
}
Observer::~Observer() {
    for(auto s : _subjects_observed) {
        s->removeObserver(this);
    }
}
void Subject::addObserver(Signal::Observer* observer) {
    _observers.push_back(observer);
    observer->_subjects_observed.push_back(this);
}
void Subject::removeObserver(Signal::Observer* observer) {
    auto itr = std::find(_observers.begin(), _observers.end(), observer);
    assert(itr != _observers.end());
    _observers.erase(itr);
}
void Subject::notify(const GameObject& entity, Signal::Event event) {
    for (auto o : _observers) {
        o->onNotify(entity, event);
    }
}
Subject::~Subject() {
    for(auto o : _observers) {
        o->_removeSubject(this);
    }
}
