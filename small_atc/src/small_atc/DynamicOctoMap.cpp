#include <small_atc/DynamicOctoMap.h>
#include <small_atc/CollisionCheckerFCL.h>
#include <octomap_msgs/conversions.h>
#include <boost/shared_ptr.hpp>
#include <fcl/octree.h>

using namespace boost;

DynamicOctoMap::DynamicOctoMap(int resolution)
    : octomap(new octomap::OcTree(resolution)) {
    shared_ptr<fcl::CollisionGeometry> geometry(new fcl::OcTree(octomap));
    setCollisionObject(new fcl::CollisionObject(geometry));
}

DynamicOctoMap::DynamicOctoMap(const std::string &filename)
    : octomap(new octomap::OcTree(filename)) {
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
	int max_x = std::max(AA.x, BB.x);
	int max_y = std::max(AA.y, BB.y);
	int max_z = std::max(AA.z, BB.z);
	int min_x = std::min(AA.x, BB.x);
	int min_y = std::min(AA.y, BB.y);
	int min_z = std::min(AA.z, BB.z);

	/*
	for (int x = min_x; x < max_x; x++)
		for (int y = min_y; y < max_y; y++)
			for (int z = min_z; z < max_z; z++)
				cloud.push_back(x, y, z);
				*/
    for (int x = min_x; x < max_x; x++)
        for (int z = min_z; z < max_z; z++) {
            cloud.push_back(x, AA.y, z); 
            cloud.push_back(x, BB.y, z); 
        }
    for (int y = min_y; y < max_y; y++)
        for (int z = min_z; z < max_z; z++) {
            cloud.push_back(AA.x, y, z); 
            cloud.push_back(BB.x, y, z); 
        }
    for (int x = min_x; x < max_x; x++)
        for (int y = min_y; y < max_y; y++) {
            cloud.push_back(x, y, AA.z); 
            cloud.push_back(x, y, BB.z); 
        }
    for (auto p : cloud)
        octomap->updateNode(p, true);
}

void DynamicOctoMap::drawSphere(const geometry_msgs::Point &center, double R)
{
    for (double fi = 0; fi < 2 * M_PI; fi += 0.1)
        for (double teta = 0; teta < 2 * M_PI; teta += 0.1) {
            octomap::point3d point(center.x + R * sin(teta) * cos(fi),
                                   center.y + R * sin(teta) * sin(fi),
                                   center.z + R * cos(teta));
            octomap->updateNode(point, true);
        }
}
