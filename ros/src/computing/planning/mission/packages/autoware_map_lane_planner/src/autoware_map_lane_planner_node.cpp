#include <ros/ros.h>
#include <autoware_map_lane_planner/autoware_map_lane_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "autoware_map_lane_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::spin();
    return 0;
}