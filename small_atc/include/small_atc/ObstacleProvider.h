#ifndef __OBSTACLE_PROVIDER_H__
#define __OBSTACLE_PROVIDER_H__

#include "HasCollision.h"
#include "DynamicMap.h"
#include <ros/ros.h>

typedef std::vector<const HasCollision*> HasCollisionVector;

class ObstacleProvider
{
public:
    /***
     * Get vector of collision objects
     **/
    virtual HasCollisionVector getCollisionObjects() const = 0;
};

template<class Obstacle>
class ObstacleProviderImpl : public ObstacleProvider
{
public:
    ObstacleProviderImpl(ros::NodeHandle &node_handle)
        : node(node_handle) {}

    ~ObstacleProviderImpl() {
        for (auto i = obstacles.begin(); i != obstacles.end(); ++i)
            delete i->second;
    }
    /***
     * All collision objects getter
     **/
    HasCollisionVector getCollisionObjects() const {
        HasCollisionVector vec;
        for (auto i = obstacles.begin(); i != obstacles.end(); ++i)
           vec.push_back(static_cast<const HasCollision*>(i->second));
        return vec;
    }

    /**
     * Obstacle registration method,
     * returns obstacle id
     **/
    void addObstacle(int id, const Obstacle *obstacle) {
        obstacles[id] = obstacle;
        std::string pub_name = "obstacle/" + std::to_string(id);
        publishers[id] = node.advertise<octomap_msgs::Octomap>(pub_name, 1);
    }

    /**
     * Remove obstacle with id from collider
     **/
    bool removeObstacle(int id) {
        delete obstacles[id];
        return obstacles.erase(id) + publishers.erase(id);
    }

    void publishAll() {
        for (auto k : obstacles) {
            int id = k.first;
            const Obstacle *map = k.second;
            publishers[id].publish(map->getOctomapMsg());
        }
    }

private:
    ros::NodeHandle &node;
    std::map<int,const Obstacle*> obstacles;
    std::map<int,ros::Publisher> publishers;
};

#endif
