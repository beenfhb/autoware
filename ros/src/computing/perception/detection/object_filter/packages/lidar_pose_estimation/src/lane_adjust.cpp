#include "vector_map.hpp"

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "autoware_msgs/DetectedObjectArray.h"

void callback_objects(const autoware_msgs::DetectedObjectArray::ConstPtr& message);

VectorMap vector_map;
ros::Subscriber sub;
ros::Publisher  pub;
tf2_ros::Buffer tf_buffer;

int main(int argc, char **argv)
{
    std::string object_topic_input;
    std::string object_topic_output;

    ros::init(argc, argv, "lidar_object_filter_lane_adjust");
    ros::NodeHandle nh("~");
    nh.param<std::string>("object_topic_input",  object_topic_input,  "/detection/lidar_detector/objects");
    nh.param<std::string>("object_topic_output", object_topic_output, "/detection/object_filter/lane_adjust/objects");

    tf2_ros::TransformListener tf_listener(tf_buffer);

    if(!vector_map.load())
    {
        ROS_ERROR("failed to load vector map");
        return -1;
    }
    ROS_INFO("succeeded to load vector map");

    sub = nh.subscribe(object_topic_input, 1, callback_objects);
    pub = nh.advertise<autoware_msgs::DetectedObjectArray>(object_topic_output, 1);

    ros::spin();

    pub.shutdown();
    sub.shutdown();
}

void callback_objects(const autoware_msgs::DetectedObjectArray::ConstPtr& message)
{
    geometry_msgs::TransformStamped tf_stamped;
    try
    {
        tf_stamped = tf_buffer.lookupTransform("map", message->header.frame_id, message->header.stamp, ros::Duration(1.0));
    }
    catch (tf2::TransformException& exception)
    {
        ROS_WARN("failed to lookup transform: %s", exception.what());
        return;
    }

    autoware_msgs::DetectedObjectArray adjusted_message;
    adjusted_message.objects.reserve(message->objects.size());
    for(const auto& object: message->objects)
    {
        geometry_msgs::Point point;
        tf2::Quaternion quaternion;
        double rx, ry, rz, yaw;
        tf2::doTransform(object.pose.position, point, tf_stamped);

        autoware_msgs::DetectedObject adjusted_object = object;
        if(vector_map.getyaw(point.x, point.y, yaw))
        {
            tf2::fromMsg(tf_stamped.transform.rotation, quaternion);
            tf2::Matrix3x3(quaternion).getRPY(rx, ry, rz);
            quaternion.setRPY(0.0, 0.0, yaw - rz);
            adjusted_object.pose.orientation = tf2::toMsg(quaternion);
        }
        else
        {
            quaternion.setRPY(0.0, -M_PI/2.0, 0.0);
            adjusted_object.pose.orientation = tf2::toMsg(quaternion);
        }
        adjusted_message.objects.emplace_back(adjusted_object);
    }
    pub.publish(adjusted_message);
}