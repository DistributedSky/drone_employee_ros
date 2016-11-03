#include <small_atc/PlannerOMPL.h>
#include <small_atc/CollisionCheckerFCL.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <fcl/shape/geometric_shapes.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

PlannerOMPL::PlannerOMPL(const ObstacleProvider *obstProvider,
                         const MapMetaData &meta)
        : Planner(meta)
        , obstacles(obstProvider)
        , space(new ob::RealVectorStateSpace(3))
        , ss(space)
{
    // Set up the bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(0, meta.dimentions[0]);
    bounds.setHigh(1, meta.dimentions[1]);
    bounds.setHigh(2, meta.dimentions[2]);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

std::vector<geometry_msgs::Point>
PlannerOMPL::plan(const geometry_msgs::Point &start,
                  const geometry_msgs::Point &goal) { 

    ob::ScopedState<ob::RealVectorStateSpace> start_space(space);
    start_space[0] = start.x;
    start_space[1] = start.y;
    start_space[2] = start.z;
    std::cout << "Start:" << std::endl;
    start_space.print(std::cout);

    ob::ScopedState<ob::RealVectorStateSpace> goal_space(space);
    goal_space[0] = goal.x;
    goal_space[1] = goal.y;
    goal_space[2] = goal.z;
    std::cout << "Goal:" << std::endl;
    goal_space.print(std::cout);

    ss.setStartAndGoalStates(start_space, goal_space);
    
    // Making the OMPL planner
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
    ob::PlannerStatus solved = ss.solve();

    std::vector<geometry_msgs::Point> path;
    if (solved) {
        ss.simplifySolution();
        std::cout << "------------" << std::endl;
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().print(std::cout);

        for (auto pt : ss.getSolutionPath().getStates()) {
            ob::State *state = &(*pt);
            typedef ob::RealVectorStateSpace::StateType type;
            const type *s = static_cast<const type *>(state);

            geometry_msgs::Point point;
            point.x = s->values[0];
            point.y = s->values[1];
            point.z = s->values[2];
            path.push_back(point);
        }
    }
    else
        std::cout << "Solution is not found" << std::endl;
    return path;
}

void PlannerOMPL::setDroneModel(std::shared_ptr<fcl::CollisionGeometry> model) {
    // Making the validity checker
    ob::StateValidityCheckerPtr checker(
            new ValidityChecker(ss.getSpaceInformation(), obstacles, model));
    ss.setStateValidityChecker(checker);
}

bool PlannerOMPL::ValidityChecker::isValid(const ob::State *state) const {
    typedef ob::RealVectorStateSpace::StateType type;
    const type *s = static_cast<const type *>(state);
    // Making an object
    fcl::Transform3f     stateTrans(fcl::Vec3f(s->values[0], s->values[1], s->values[2]));
    fcl::CollisionObject stateObject(drone_model, stateTrans);
    HasCollisionFCL      collisionObject(&stateObject);
    // Check collision for all obstacles in collider
    for (auto obstacle : collider->getCollisionObjects())
        if (obstacle->hasCollision(&collisionObject))
            return false;
    return true;
}

