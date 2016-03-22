#include "small_atc/DynamicMap.h" 
#include <yaml-cpp/yaml.h>

void DynamicMap::drawRoute(const std::vector<geometry_msgs::Point> &route, double R) {
    for (size_t i = 0; i < route.size() - 1; i++) {
        geometry_msgs::Point a = route[i],
                             b = route[i+1];
        double distance = sqrt(pow(b.x - a.x, 2) +
                               pow(b.y - a.y, 2) +
                               pow(b.z - a.z, 2)),
               dx = (b.x - a.x) / distance,
               dy = (b.y - a.y) / distance,
               dz = (b.z - a.z) / distance;
        geometry_msgs::Point center;
        for (int j = 0; j < distance; j += 4) {
            center.x = a.x + j * dx, 
            center.y = a.y + j * dy,
            center.z = a.z + j * dz;
            drawSphere(center, R);
        }
    }
}

MapMetaData DynamicMap::readMeta(const std::string &filename) {
    YAML::Node node = YAML::LoadFile(filename);
    meta.resolution = node["resolution"].as<double>();
    meta.origin_latitude  = node["origin_latitude"].as<double>();
    meta.origin_longitude = node["origin_longitude"].as<double>();
    meta.origin_altitude  = node["origin_altitude"].as<double>();
    meta.dimentions[0] = node["dimentions"][0].as<double>();
    meta.dimentions[1] = node["dimentions"][1].as<double>();
    meta.dimentions[2] = node["dimentions"][2].as<double>();
}

bool DynamicMap::writeMeta(const std::string &filename) {
    YAML::Node node;
    node["resolution"] = meta.resolution;
    node["origin_latitude"]  = meta.origin_latitude;
    node["origin_longitude"] = meta.origin_longitude;
    node["origin_altitude"]  = meta.origin_altitude;
    YAML::Node dims;
    dims.push_back(meta.dimentions[0]);
    dims.push_back(meta.dimentions[1]);
    dims.push_back(meta.dimentions[2]);
    node["dimentions"] = dims;
    std::ofstream fout(filename);
    fout << node;
}
