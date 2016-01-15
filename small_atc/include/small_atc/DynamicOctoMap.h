#ifndef __DYNAMIC_OCTO_MAP_H
#define __DYNAMIC_OCTO_MAP_H

#include "DynamicMap.h" 
#include "CollisionCheckerFCL.h" 

#include <octomap/octomap.h>
#include <fcl/collision.h>

class DynamicOctoMap :
    public DynamicMap,
    public HasCollisionImpl<fcl::CollisionObject,CollisionCheckerFCL>
{
public:
    DynamicOctoMap(int resolution=3);
    DynamicOctoMap(const std::string &filename);
    
    /***
     * Octomap message
     * Used by visualisation
     **/
    octomap_msgs::Octomap getOctomapMsg() const; 

    /***
     * Map resolution getter
     **/
    double getResolution() const
    { return octomap->getResolution(); }

    /***
     * Map update methods
     **/
    void drawAABB(const geometry_msgs::Point &AA,
                  const geometry_msgs::Point &BB);
    void drawSphere(const geometry_msgs::Point &center, double R);

protected:
    boost::shared_ptr<octomap::OcTree> octomap;
};

#endif
