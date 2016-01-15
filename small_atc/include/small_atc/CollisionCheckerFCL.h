#ifndef __COLLISION_CHECKER_FCL_H__
#define __COLLISION_CHECKER_FCL_H__

#include "CollisionChecker.h" 
#include "HasCollision.h" 
#include <fcl/collision.h>

class CollisionCheckerFCL :
    public CollisionChecker<fcl::CollisionObject>
{
public:
    static CollisionChecker<fcl::CollisionObject> * instance() {
        static CollisionChecker<fcl::CollisionObject> *inst;
        if (inst == nullptr)
            inst = new CollisionCheckerFCL;
        return inst;
    }

    bool collide(const fcl::CollisionObject *a, const fcl::CollisionObject *b) const {
        fcl::CollisionRequest req;
        fcl::CollisionResult res;
        return fcl::collide(a, b, req, res);
    }
};

typedef HasCollisionImpl<fcl::CollisionObject, CollisionCheckerFCL> HasCollisionFCL;

#endif
