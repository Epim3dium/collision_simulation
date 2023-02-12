#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/System/Vector2.hpp"

namespace EPI_NAMESPACE {

typedef sf::Vector2f vec2f;

#define EPI_VIRTUAL_SIZE 8
#define EPI_CAST_POSPROXY(posproxy)        (*(Transform*)(&posproxy   - EPI_VIRTUAL_SIZE))
#define EPI_CAST_ROTATIONPROXY(scaleproxy) (*(Transform*)(&scaleproxy - EPI_VIRTUAL_SIZE - 1))
#define EPI_CAST_SCALEPROXY(scaleproxy)    (*(Transform*)(&scaleproxy - EPI_VIRTUAL_SIZE - 1 - 1))

class Transform {
public:
    struct PosProxy{
        vec2f operator()() const {
            return EPI_CAST_POSPROXY(*this).getPos();
        }
        void operator =(const vec2f& right) {
            return EPI_CAST_POSPROXY(*this).setPos(right);
        }
    }position;
    struct RotProxy{
        float operator()() const {
            return EPI_CAST_ROTATIONPROXY(*this).getRot();
        }
        void operator =(const float& right) {
            return EPI_CAST_ROTATIONPROXY(*this).setRot(right);
        }
    }rotation;
    struct ScaleProxy{
        vec2f operator()() const {
            return EPI_CAST_SCALEPROXY(*this).getScale();
        }
        void operator =(const vec2f& right) {
            return EPI_CAST_SCALEPROXY(*this).setScale(right);
        }
    }scale;
private:
    vec2f m_position;
    float m_rotation;
    vec2f m_scale;
public:
    virtual vec2f getPos() const {return m_position; }
    virtual void setPos(vec2f v) {m_position = v; }

    virtual vec2f getScale() const {return m_scale; }
    virtual void setScale(vec2f v) {m_scale = v; }

    virtual float getRot() const {return m_rotation; }
    virtual void setRot(float r) {m_rotation = r; }
};



#define PROXYOPERATORS(PROXY, type)\
static inline type operator -(const Transform::PROXY##Proxy & right) {\
    return -(right());\
}\
static inline void operator +=(Transform::PROXY##Proxy& left, const type& right) {\
    left = (left() + right);\
}\
static inline void operator +=(type& right, const Transform::PROXY##Proxy& left) {\
    right += left();\
}\
static inline void operator -=(Transform::PROXY##Proxy& left, const type& right) {\
    left = left() - right;\
}\
static inline void operator -=(type& left, const Transform::PROXY##Proxy& right) {\
    left -= right();\
}\
static inline type operator +(const Transform::PROXY##Proxy& left, const type& right) {\
    return left() + right;\
}\
static inline type operator +(type& left, Transform::PROXY##Proxy& right) {\
    return left + right();\
}\
static inline type operator -(const Transform::PROXY##Proxy& left, const type& right) {\
    return left() - right;\
}\
static inline type operator -(type& left, Transform::PROXY##Proxy& right) {\
    return left - right();\
}\
template<class T>\
static inline type operator *(const Transform::PROXY##Proxy& left, T right) {\
    return left() * right;\
}\
template<class T>\
static inline type operator *(T left, const Transform::PROXY##Proxy& right) {\
    return right() * left;\
}\
\
template<class T>\
static inline void operator *=(Transform::PROXY##Proxy& left, T right) {\
    left = left() * right;\
}\
template<class T>\
static inline type operator /(const Transform::PROXY##Proxy& left, T right) {\
    return left() / right;\
}\
template<class T>\
static inline void operator /=(Transform::PROXY##Proxy& left, T right) {\
    left = left / right;\
}\
static inline bool operator ==(const Transform::PROXY##Proxy& left, const type& right) {\
    return left() == right;\
}\
static inline bool operator ==(type& left, Transform::PROXY##Proxy& right) {\
    return right() == left;\
}\
static inline bool operator !=(const Transform::PROXY##Proxy& left, const type& right) {\
    return left() != right;\
}\
static inline bool operator !=(type& left, Transform::PROXY##Proxy& right) {\
    return right() != left;\
}\

PROXYOPERATORS(Pos, vec2f);
PROXYOPERATORS(Rot, float);
PROXYOPERATORS(Scale, vec2f);

}
