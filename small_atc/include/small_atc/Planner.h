#ifndef __PLANNER_H__
#define __PLANNER_H__ 

#include "DynamicMap.h" 
#include <vector>
#include <geometry_msgs/Point.h>

using namespace geometry_msgs;

class Planner
{
public:
    Planner(const MapMetaData &meta) : map_meta(meta) {}
    virtual ~Planner() {};

    /*** Map meta getter ***/
    MapMetaData getMapMetaData() const
    { return map_meta; }

    /*** Planning in map coords ***/
    virtual std::vector<Point> plan(const Point &start,
                                    const Point &goal) = 0; 

protected:
    MapMetaData map_meta;
};

#endif // __PLANNER_H__
