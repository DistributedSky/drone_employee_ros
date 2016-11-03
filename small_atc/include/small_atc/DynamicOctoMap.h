#ifndef __DYNAMIC_OCTO_MAP_H
#define __DYNAMIC_OCTO_MAP_H

#include "DynamicMap.h" 
#include "CollisionCheckerFCL.h" 

#include <octomap/octomap.h>
#include <fcl/collision.h>
#include <memory>

class DynamicOctoMap :
    public DynamicMap,
    public HasCollisionImpl<fcl::CollisionObject,CollisionCheckerFCL>
{
public:
    DynamicOctoMap(int resolution=3,
        double max_x=10, double max_y=10, double max_z=10,
        double latitude=0, double longitude=0, double altitude=0);
    DynamicOctoMap(const std::string &filename);
    
    /***
     * Octomap message
     * Used by visualisation
     **/
    octomap_msgs::Octomap getOctomapMsg() const; 

    /***
     * Map update methods
     **/
    void drawAABB(const geometry_msgs::Point &AA,
                  const geometry_msgs::Point &BB);
    void drawSphere(const geometry_msgs::Point &center, double R);


    /***
     * Dump map as binary file
     **/
    bool writeBinary(const std::string &filename) {return false;}
    //{ return octomap->writeBinary(filename); }

protected:
    std::shared_ptr<const octomap::OcTree> octomap;
};

#endif
