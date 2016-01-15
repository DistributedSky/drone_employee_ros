#include <small_atc/SmallATC.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "small_atc");

    SmallATC atc;
    return atc.exec();
}
