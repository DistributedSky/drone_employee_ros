#ifndef __SMALL_ATC_H__
#define __SMALL_ATC_H__

#include "Planner.h" 
#include "DynamicOctoMap.h" 
#include "ObstacleProvider.h" 

#include <ros/ros.h>
#include <small_atc_msgs/LocalRouteRequest.h>

using namespace ros;
using namespace small_atc_msgs;

class SmallATC
{
public:
    SmallATC();
    ~SmallATC();

    /**
     * Exec the Small Air Traffic Controller
     **/
    int exec();

protected:
    void requestHandler(const LocalRouteRequest::ConstPtr &msg);

private:
    ObstacleProviderImpl<DynamicOctoMap> *obstacles;
    Planner    *atc_planner;
    NodeHandle  node_handle;

    Publisher   route_response;
    Subscriber  route_request;
};

#endif
