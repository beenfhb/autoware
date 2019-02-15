
#include <ros/ros.h>
#include <opendrive2autoware_converter/opendrive2autoware_converter_core.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opendrive2autoware_converter");

    autoware_map::OpenDrive2AutoConv converter;
    autoware_map::InternalRoadNet map;
    converter.loadOpenDRIVE("/home/hatem/open_drive/sample1.1.xodr", map);
    ros::spin();
}
