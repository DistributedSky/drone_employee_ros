#include <boost/program_options.hpp>
#include <liblas/liblas.hpp>
#include <octomap/octomap.h>
#include <iostream>

using namespace boost;
using namespace liblas;
using namespace octomap;

int main(int argc, const char *argv[])
{
    // Parse input arguments
    program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input,i", program_options::value<std::string>(), "set .LAS input filename")
        ("output,o", program_options::value<std::string>(), "set .bt output filename")
        ("resolution,r", program_options::value<int>(), "set map resolution in meters")
        ;

    program_options::variables_map vm;
    program_options::store(program_options::parse_command_line(argc, argv, desc), vm);
    program_options::notify(vm);    

    if (vm.count("help") || !vm.count("input") ||
       !vm.count("output") || !vm.count("resolution")) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    std::string points_filename = vm["input"].as<char*>();
    std::string map_filename = vm["output"].as<char*>();
    int resolution = vm["resolution"].as<int>();
    
    // Create new map
    shared_ptr<OcTree> map(new OcTree(resolution));

    // Open LAS file
    shared_ptr<std::istream> las(Open(points_filename,
                                      std::ios::in | std::ios::binary));
    if (las == NULL)
    {
        std::cout << "Unable to open "
                  << points_filename << std::endl;
        return 1;
    }

    // Make a LAS reader and get a header
    shared_ptr<Reader> las_reader(new Reader(*las));
    Header las_header = las_reader->GetHeader();

    // Zero point of map
    point3d offset(las_header.GetMinX(),
                   las_header.GetMinY(),
                   las_header.GetMinZ());

    // Read full LAS file
    while (las_reader->ReadNextPoint())
    {
        // Take 3D point
        Point las_point = las_reader->GetPoint();
        point3d point(las_point.GetX(), las_point.GetY(), las_point.GetZ());
        // Shift point to zero
        point -= offset;
        map->updateNode(point, true, true);
    }
    // Map optimization
    map->updateInnerOccupancy();
    // Dump map to file
    if (!map->writeBinary(map_filename))
    {
        std::cout << "Unable to write map "
                  << map_filename << std::endl;
        return 1;
    }
    return 0;
}
