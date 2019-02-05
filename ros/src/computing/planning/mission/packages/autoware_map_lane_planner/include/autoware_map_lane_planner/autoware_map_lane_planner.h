#ifndef AUTOWARE_MAP_LANE_PLANNER_H_INCLUDED
#define AUTOWARE_MAP_LANE_PLANNER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Int32.h>

//headers in Autoware
#include <autoware_map/autoware_map.h>
#include <geometry_msgs/PoseStamped.h>

//headers in Boost
#include <boost/optional.hpp>

class AutowareMapLanePlanner
{
public:
    AutowareMapLanePlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AutowareMapLanePlanner();
private:
    autoware_map::AutowareMap autoware_map_;
    /* ROS related members */
    void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr msg);
    ros::Subscriber current_pose_sub_;
    ros::Publisher closest_waypoint_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    /* values used in Autoware Map Lane Planner */
    geometry_msgs::PoseStamped current_pose_;
    /* parameters for Autoware Map Lane Planner*/
    double search_radius_;
    /* functions */
    bool findClosestWaypointCandidates(autoware_map_msgs::Waypoint waypoint);
    boost::optional<autoware_map_msgs::Waypoint> findClosestWaypoint();
    double getDiffAngle(double from,double to);
    void getRPY(geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw);
};

#endif