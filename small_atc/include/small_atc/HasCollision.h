#ifndef __HAS_COLLISION_H__
#define __HAS_COLLISION_H__

#include "CollisionChecker.h" 

class HasCollision
{
public:
    /**
     * Collision checking with two objects
     **/
    virtual bool hasCollision(const HasCollision *object) const = 0;
};

template<class Object, class Collider>
class HasCollisionImpl : public HasCollision
{
public:
    HasCollisionImpl(Object *object = nullptr)
        : collision_object(object)
    {}

    /***
    * Collision check
    **/
    virtual bool hasCollision(const HasCollision *object) const
    { 
        auto inst = static_cast<const HasCollisionImpl<Object,Collider>*>(object);
        return Collider::instance()->collide(collision_object, inst->collision_object);
    }

protected:
    /***
    * Collision object setter
    **/
    void setCollisionObject(Object *object)
    { collision_object = object; }

private:
    Object *collision_object;
};

#endif
