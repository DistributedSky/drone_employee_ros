#include <small_atc/DynamicOctoMap.h>
#include <small_atc/CollisionCheckerFCL.h>
#include <octomap_msgs/conversions.h>
#include <boost/shared_ptr.hpp>
#include <fcl/octree.h>

using namespace boost;

DynamicOctoMap::DynamicOctoMap(int resolution,
        double max_x, double max_y, double max_z,
        double latitude, double longitude, double altitude)
    : octomap(new octomap::OcTree(resolution)) {
    shared_ptr<fcl::CollisionGeometry> geometry(new fcl::OcTree(octomap));
    setCollisionObject(new fcl::CollisionObject(geometry));

    meta.resolution = resolution;
    meta.origin_latitude  = latitude;
    meta.origin_longitude = longitude;
    meta.origin_altitude  = altitude;
    meta.dimentions[0] = max_x;
    meta.dimentions[1] = max_y;
    meta.dimentions[2] = max_z;
}

DynamicOctoMap::DynamicOctoMap(const std::string &filename)
    : DynamicMap(filename)
    , octomap(new octomap::OcTree(filename + ".bt")) {
    shared_ptr<fcl::CollisionGeometry> geometry(new fcl::OcTree(octomap));
    setCollisionObject(new fcl::CollisionObject(geometry));
}

octomap_msgs::Octomap DynamicOctoMap::getOctomapMsg() const 
{
    octomap_msgs::Octomap msg;
    octomap_msgs::fullMapToMsg(*octomap, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    return msg; 
}

void DynamicOctoMap::drawAABB(const geometry_msgs::Point &AA,
                              const geometry_msgs::Point &BB)
{
    octomap::Pointcloud cloud;
	double max_x = std::max(AA.x, BB.x);
	double max_y = std::max(AA.y, BB.y);
	double max_z = std::max(AA.z, BB.z);
	double min_x = std::min(AA.x, BB.x);
	double min_y = std::min(AA.y, BB.y);
	double min_z = std::min(AA.z, BB.z);
    double step = meta.resolution / 2.0;

	/*
	for (int x = min_x; x < max_x; x++)
		for (int y = min_y; y < max_y; y++)
			for (int z = min_z; z < max_z; z++)
				cloud.push_back(x, y, z);
				*/
    for (double x = min_x; x <= max_x; x += step)
        for (double z = min_z; z <= max_z; z += step) {
            cloud.push_back(x, AA.y, z); 
            cloud.push_back(x, BB.y, z); 
        }
    for (double y = min_y; y <= max_y; y += step)
        for (double z = min_z; z <= max_z; z += step) {
            cloud.push_back(AA.x, y, z); 
            cloud.push_back(BB.x, y, z); 
        }
    for (double x = min_x; x <= max_x; x += step)
        for (double y = min_y; y <= max_y; y += step) {
            cloud.push_back(x, y, AA.z); 
            cloud.push_back(x, y, BB.z); 
        }
    for (auto p : cloud)
        octomap->updateNode(p, true, false);
    octomap->updateInnerOccupancy();
}

void DynamicOctoMap::drawSphere(const geometry_msgs::Point &center, double R)
{
    double step = meta.resolution * 0.01;
    for (double fi = 0; fi <= 2 * M_PI; fi += step)
        for (double teta = 0; teta <= 2 * M_PI; teta += step) {
            octomap::point3d point(center.x + R * sin(teta) * cos(fi),
                                   center.y + R * sin(teta) * sin(fi),
                                   center.z + R * cos(teta));
            octomap->updateNode(point, true, false);
        }
    octomap->updateInnerOccupancy();
}
