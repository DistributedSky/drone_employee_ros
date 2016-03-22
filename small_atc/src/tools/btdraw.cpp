#include <octomap/octomap.h>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include "small_atc/DynamicOctoMap.h" 

namespace po = boost::program_options;
using namespace boost;

inline void draw_cube(shared_ptr<DynamicOctoMap> &map, const std::string &params)
{
    // Parse params
    std::stringstream ss(params);
    geometry_msgs::Point AA;
    geometry_msgs::Point BB;
    ss >> AA.x >> AA.y >> AA.z;
    ss >> BB.x >> BB.y >> BB.z;
    // Drawing
    map->drawAABB(AA, BB);
}

inline void draw_sphere(shared_ptr<DynamicOctoMap> &map, const std::string &params)
{
    // Parse params
    std::stringstream ss(params);
    geometry_msgs::Point center;
    double radius;
    ss >> center.x >> center.y >> center.z >> radius;
    // Drawing
    map->drawSphere(center, radius);
}

int main(int argc, const char *argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", po::value<std::string>()->default_value("out"), "output map file")
        ("resolution,r", po::value<int>()->default_value(5), "set map resolution in meters")
        ("input,i", po::value<std::string>(), "input map file")
        ("cube", po::value<std::string>(), "draw a cube by AABB points")
        ("sphere", po::value<std::string>(), "draw a sphere by center point and radius")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    shared_ptr<DynamicOctoMap> map; 
    // Load or create map
    if (vm.count("input"))
        map = shared_ptr<DynamicOctoMap>(new DynamicOctoMap(vm["input"].as<std::string>()));
    else
        map = shared_ptr<DynamicOctoMap>(new DynamicOctoMap(vm["resolution"].as<int>()));

    // Drawing
    if (vm.count("cube"))
        draw_cube(map, vm["cube"].as<std::string>());

    if (vm.count("sphere"))
        draw_sphere(map, vm["sphere"].as<std::string>());

    // Dump map
    if (!map->write(vm["output"].as<std::string>()))
    {
        std::cout << "Unable to write map "
                  << vm["output"].as<std::string>() << std::endl;
        return 1;
    }
    
    return 0;
}
