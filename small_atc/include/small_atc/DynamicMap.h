#ifndef __DYNAMIC_MAP_H__
#define __DYNAMIC_MAP_H__

#include <octomap_msgs/conversions.h>
#include <small_atc_msgs/SatFix.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Point.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

struct MapMetaData {
    double resolution;
    double origin_latitude;
    double origin_longitude;
    double origin_altitude;
    double dimentions[3];
};

class DynamicMap
{
public:
    DynamicMap(const std::string &meta_filename="")
    { if (meta_filename != "") readMeta(meta_filename); } 
    virtual ~DynamicMap() {}

    /***
     * Octomap message
     * Used by visualisation
     **/
    virtual octomap_msgs::Octomap getOctomapMsg() const = 0; 

    /***
     * Map metadata getter
     **/
    MapMetaData getMeta() const
    { return meta; }

    /***
     * Map update methods
     **/
    virtual void drawAABB(const geometry_msgs::Point &AA,
                          const geometry_msgs::Point &BB) = 0;
    virtual void drawSphere(const geometry_msgs::Point &center, double R) = 0;
    virtual void drawRoute(const std::vector<geometry_msgs::Point> &route, double R); 

    /***
     * Read metadata from file
     **/
    virtual MapMetaData readMeta(const std::string &filename);

    /***
     * Dump map as binary file
     **/
    virtual bool writeBinary(const std::string &filename) = 0;

    /***
     * Dump map metadata file
     **/
    virtual bool writeMeta(const std::string &filename);

    /***
     * Dump full map into file
     **/
    virtual bool write(const std::string &filename)
    {
        return writeBinary(filename + ".bt")
            && writeMeta(filename + ".yaml");
    }

protected:
    MapMetaData meta;
};

#endif // __DYNAMIC_MAP_H__
