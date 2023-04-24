#ifndef GAME_OBJECT_UTILS_H
#define GAME_OBJECT_UTILS_H
#include <algorithm>
#include <cstddef>
#include <sys/_types/_size_t.h>
#include <vector>
#include <memory>
#include <map>
#include <string>
#include <set>
#include <list>

namespace epi {

struct GameObject;

//glorified set
class Tag {
    std::set<std::string> _tags;
public:
    inline std::vector<std::string> getList() const {
        return std::vector<std::string>(_tags.begin(), _tags.end());
    }
    inline void add(std::string t) {
        _tags.insert(t);
    }
    inline void remove(std::string t) {
        _tags.erase(t);
    }
    inline bool contains(std::string t) const {
        return _tags.contains(t);
    }
    inline bool operator==(std::string t) const {
        return contains(t);
    }
    inline bool operator==(const Tag& t) const {
        std::vector<std::string> common_data;
        set_intersection(_tags.begin(), _tags.end(), t._tags.begin(), t._tags.end(), std::back_inserter(common_data));
        return common_data.size() != 0;
    }
    inline bool operator!=(const char* t) const {
        return !(*this == t);
    }
    inline size_t size() const { return _tags.size(); };
    Tag() {}
    Tag(std::initializer_list<std::string> inits) : _tags(inits) {}
};
namespace GameObjectUtils {

//a container used for storing diffrent components of GameObject
class ChildContainer {
    std::vector<GameObject*> _array_pointers;
    std::map<size_t, GameObject*> _typed_members;
public:
    void insert(GameObject* obj);
    void erase(GameObject* obj);

    inline std::vector<GameObject*>::iterator begin() { return _array_pointers.begin(); }
    inline std::vector<GameObject*>::iterator end() { return _array_pointers.end(); }
    inline std::vector<GameObject*>::reverse_iterator rbegin() { return _array_pointers.rbegin(); }
    inline std::vector<GameObject*>::reverse_iterator rend() { return _array_pointers.rend(); }
    inline std::vector<GameObject*>::const_iterator cbegin() const { return _array_pointers.cbegin(); }
    inline std::vector<GameObject*>::const_iterator cend() const { return _array_pointers.cend(); }

    template<class T>
    T& getComponent();
    ChildContainer();
};

template <class T>
T& ChildContainer::getComponent() {
    auto h = typeid(T).hash_code();
    auto itr = _typed_members.find(h);
    assert(itr != _typed_members.end());
    return *dynamic_cast<T*>(itr->second);
}

}

namespace Signal {
//base data that is being sent between Subject and Observer
typedef std::string Event;

static constexpr const char* EventDestroyed = "destroyed";
static constexpr const char* EventChanged   = "changed";
static constexpr const char* EventInput   = "input";

struct Subject;

//abstract class used for listening to Subject's signals
class Observer {
    std::vector<Subject*> _subjects_observed;
    void _removeSubject(const Subject*);
public:
    virtual void onNotify(const GameObject& obj, Event event) = 0;
    virtual ~Observer();
    friend Subject;
};

//class used for being able to notify other observers
class Subject {
private:
    std::vector<Observer*> _observers;
public:
    void addObserver(Observer* observer);
    void removeObserver(Observer* observer);

    void notify(const GameObject& entity, Event event);
    virtual ~Subject();
};
}


}
#endif
