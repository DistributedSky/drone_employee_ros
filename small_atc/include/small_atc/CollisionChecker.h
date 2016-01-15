#ifndef __COLLISION_CHECKER_H__
#define __COLLISION_CHECKER_H__

template<class Object>
class CollisionChecker
{
public:
    //static CollisionChecker<Object> * instance() = 0;
    virtual bool collide(const Object *a, const Object *b) const = 0;
};

#endif // __COLLISION_CHECKER_H__
