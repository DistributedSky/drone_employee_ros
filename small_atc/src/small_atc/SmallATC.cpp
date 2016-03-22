#include <small_atc/SmallATC.h>
#include <small_atc/PlannerOMPL.h>
#include <small_atc/DynamicOctoMap.h>
#include <small_atc_msgs/LocalRouteResponse.h>
#include <octomap_msgs/Octomap.h>

using namespace ros;
using namespace small_atc_msgs;

SmallATC::SmallATC() {
    std::string filename;
    param::get("~map_file", filename);
    topomap = new DynamicOctoMap(filename);
    obstacles->addObstacle(0, topomap); 
    ROS_INFO("Loaded topographic map: %s", filename.c_str());
    // Register obstacle
    obstacles = new ObstacleProviderImpl<DynamicOctoMap>(node_handle);
    // Make a planner
    atc_planner = new PlannerOMPL(obstacles, topomap->getMeta());
    // Making the route request/response handlers
    route_response = node_handle.advertise<LocalRouteResponse>("route/response_local", 1);
    route_request  = node_handle.subscribe<LocalRouteRequest>("route/request_local", 1, 
            &SmallATC::requestHandler, this); 
}

SmallATC::~SmallATC() {
    delete atc_planner;
    delete obstacles;
}

void SmallATC::requestHandler(const LocalRouteRequest::ConstPtr &msg) { 
    ROS_INFO("Start planning...");
    LocalRouteResponse response;
    for (int i = 0; i < msg->checkpoints.size() - 1; i++) {
        auto plan = atc_planner->plan(msg->checkpoints[i],
                                      msg->checkpoints[i+1]);
        response.route.insert(response.route.end(), plan.begin(), plan.end());
    }
    response.valid = response.route.size() > 0;
    response.id = msg->id;
    if (response.valid) {
        ROS_INFO("Plan is valid.");
        // Register route
        DynamicOctoMap *map = new DynamicOctoMap(topomap->getMeta().resolution); 
        map->drawRoute(response.route, 5);
        obstacles->addObstacle(response.id, map);
        ROS_INFO("Plan registered with id=%d", response.id);
    } else {
        ROS_WARN("No valid plan!");
    }
    // Publish response
    route_response.publish(response);
}

int SmallATC::exec() {
    // No need to publish obstacles in this demo
    ros::spin();
    return 0;

    /*
    ros::Rate cycle(0.1);
    while (ros::ok()) {
        // Publish all the obstacles with associated topics
        obstacles->publishAll();
        ros::spinOnce();
        cycle.sleep();
    }
    return 0;
    */
}
