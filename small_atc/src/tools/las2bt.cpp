#include <liblas/liblas.hpp>
#include <octomap/octomap.h>
#include <iostream>

#define USAGE "USAGE: " << argv[0] \
    << " points.las map.bt RESOLUTION"

typedef struct {
    std::string points_filename;
    std::string map_filename;
    double resolution;
} args_t;

bool parse_args(int argc, const char *argv[], args_t &args)
{
    if (argc != 4) return false;

    args.points_filename = std::string(argv[1]);
    args.map_filename    = std::string(argv[2]);

    std::istringstream resolution(argv[3]);
    resolution >> args.resolution;

    return true;
}

using namespace boost;
using namespace liblas;
using namespace octomap;

int main(int argc, const char *argv[])
{
    // Parse input arguments
    args_t args;
    if (!parse_args(argc, argv, args))
    {
        std::cout << USAGE << std::endl;
        return 1;
    }
    
    // Create new map
    shared_ptr<OcTree> map(new OcTree(args.resolution));

    // Open LAS file
    shared_ptr<std::istream> las(Open(args.points_filename,
                                      std::ios::in | std::ios::binary));
    if (las == NULL)
    {
        std::cout << "Unable to open "
                  << args.points_filename << std::endl;
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
    if (!map->writeBinary(args.map_filename))
    {
        std::cout << "Unable to write map "
                  << args.map_filename << std::endl;
        return 1;
    }
    return 0;
}
