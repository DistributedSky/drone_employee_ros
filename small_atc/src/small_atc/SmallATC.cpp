#include <small_atc/SmallATC.h>
#include <small_atc/PlannerOMPL.h>
#include <small_atc/DynamicOctoMap.h>
#include <small_atc_msgs/LocalRouteResponse.h>
#include <octomap_msgs/Octomap.h>

using namespace ros;
using namespace small_atc_msgs;

SmallATC::SmallATC() {
    MapMetaData map_meta;
    // TODO: Load map metadata
    obstacles = new ObstacleProviderImpl<DynamicOctoMap>(node_handle);
    atc_planner = new PlannerOMPL(obstacles, map_meta);
    // Making the route request/response handlers
    route_response = node_handle.advertise<LocalRouteResponse>("route/response_local", 1);
    route_request  = node_handle.subscribe<LocalRouteRequest>("route/request_local", 1, 
            &SmallATC::requestHandler, this); 
    // Adding the first obstacle - the topographic map
    std::string filename;
    if (param::get("~map_file", filename)) {
		//Next code is crutch
		//Create restricted area
		DynamicOctoMap *obstacle = new DynamicOctoMap();
		geometry_msgs::Point point1;
		geometry_msgs::Point point2;
		point1.x = -1200;
		point1.y = -1000;
		point1.z = 1500;
		point2.x = 200;
		point2.y = -600;
		point2.z = 1200;
		obstacle->drawAABB(point1, point2);
		obstacles->addObstacle(777, obstacle);

        obstacles->addObstacle(0, new DynamicOctoMap(filename));
        ROS_INFO("Loaded topographic map: %s", filename.c_str());
    }
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
        DynamicOctoMap *map = new DynamicOctoMap(); // TODO: resolution set
        map->drawRoute(response.route, 25);
        obstacles->addObstacle(response.id, map);
        ROS_INFO("Plan registered with id=%d", response.id);
    } else {
        ROS_WARN("No valid plan!");
    }
    // Publish response
    route_response.publish(response);
}

int SmallATC::exec() {
    ros::Rate cycle(0.2);
    while (ros::ok()) {
        // Publish all the obstacles with associated topics
		//ROS_INFO("Rate");
        obstacles->publishAll();
        ros::spinOnce();
        cycle.sleep();
    }
    return 0;
}
