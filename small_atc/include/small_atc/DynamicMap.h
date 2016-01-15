#ifndef __DYNAMIC_MAP_H__
#define __DYNAMIC_MAP_H__

#include <geometry_msgs/Point.h>
#include <small_atc_msgs/SatFix.h>
#include <octomap_msgs/Octomap.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

struct MapMetaData {
    double resolution;
    uint32_t range[3];
    small_atc_msgs::SatFix origin;
};

class DynamicMap
{
public:
    virtual ~DynamicMap() {}

    /***
     * Octomap message
     * Used by visualisation
     **/
    virtual octomap_msgs::Octomap getOctomapMsg() const = 0; 

    /***
     * Map resolution getter
     **/
    virtual double getResolution() const = 0;

    /***
     * Map update methods
     **/
    virtual void drawAABB(const geometry_msgs::Point &AA,
                          const geometry_msgs::Point &BB) = 0;
    virtual void drawSphere(const geometry_msgs::Point &center, double R) = 0;

    virtual void drawRoute(const std::vector<geometry_msgs::Point> &route, double R) {
        for (size_t i = 0; i < route.size() - 1; i++) {
            geometry_msgs::Point a = route[i],
                                 b = route[i+1];
            double distance = sqrt(pow(b.x - a.x, 2) +
                                   pow(b.y - a.y, 2) +
                                   pow(b.z - a.z, 2)),
                   dx = (b.x - a.x) / distance,
                   dy = (b.y - a.y) / distance,
                   dz = (b.z - a.z) / distance;
            geometry_msgs::Point center;
            for (int j = 0; j < distance; j += 4) {
                center.x = a.x + j * dx, 
                center.y = a.y + j * dy,
                center.z = a.z + j * dz;
                drawSphere(center, R);
            }
        }
    }
};

#endif // __DYNAMIC_MAP_H__
