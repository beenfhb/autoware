#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>





class Amcl3dNode
{
public:
  Amcl3dNode();
  ~Amcl3dNode(){};

private: // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pf_pub_;
  ros::Publisher current_pose_pub_;
  ros::Subscriber init_pose_sub_; // initial pose
  ros::Timer publish_timer_;      // publish timer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg);
  void ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &input_ndt_pose_msg);
  void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg);
  void publishTimerCallback(const ros::TimerEvent &e);

private:
  std::shared_ptr<Amcl> amcl_;
  std::string world_frame_id_;
  // std::string map_frame_id_;
  // std::string base_link_frame_id_;
  // std::string odom_frame_id_;
  std::shared_ptr<FooPredictionModelNode> prediction_model_node_;
};

Amcl3dNode::Amcl3dNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  // amcl param
  {
    AmclParam amcl_param;
    pnh_.param<double>("amcl_param/augmented_mcl/alpha_fast", amcl_param.augmented_mcl.alpha_fast, double(0.9));
    pnh_.param<double>("amcl_param/augmented_mcl/alpha_slow", amcl_param.augmented_mcl.alpha_slow, double(0.1));
    pnh_.param<double>("amcl_param/augmented_mcl/w_fast", amcl_param.augmented_mcl.w_fast, double(0.5));
    pnh_.param<double>("amcl_param/augmented_mcl/w_slow", amcl_param.augmented_mcl.w_slow, double(0.5));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_x_var", amcl_param.augmented_mcl.noise_x_var, double(1.0));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_y_var", amcl_param.augmented_mcl.noise_y_var, double(1.0));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_z_var", amcl_param.augmented_mcl.noise_z_var, double(0.2));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_roll_var", amcl_param.augmented_mcl.noise_roll_var, double(0.1));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_pitch_var", amcl_param.augmented_mcl.noise_pitch_var, double(0.3));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_yaw_var", amcl_param.augmented_mcl.noise_yaw_var, double(1.0));
    pnh_.param<double>("amcl_param/resample_timing/ess_ratio_threshold", amcl_param.resample_timing.ess_ratio_threshold, double(0.9));
    int initial_particle_num;
    pnh_.param<int>("amcl_param/init_pose/initial_particle_num", initial_particle_num, int(10));
    amcl_param.init_pose.initial_particle_num = (size_t)(initial_particle_num);
    int min_particle_num;
    pnh_.param<int>("amcl_param/kld_sampling/min_particle_num", min_particle_num, int(10));
    amcl_param.kld_sampling.min_particle_num = (size_t)(min_particle_num);
    int max_particle_num;
    pnh_.param<int>("amcl_param/kld_sampling/max_particle_num", max_particle_num, int(10));
    amcl_param.kld_sampling.max_particle_num = (size_t)(max_particle_num);
    pnh_.param<double>("amcl_param/kld_sampling/delta", amcl_param.kld_sampling.delta, double(0.5));
    pnh_.param<double>("amcl_param/kld_sampling/epsilon", amcl_param.kld_sampling.epsilon, double(0.5));
    pnh_.param<double>("amcl_param/kld_sampling/x_bin_width", amcl_param.kld_sampling.x_bin_width, double(0.2));
    pnh_.param<double>("amcl_param/kld_sampling/y_bin_width", amcl_param.kld_sampling.y_bin_width, double(0.2));
    pnh_.param<double>("amcl_param/kld_sampling/z_bin_width", amcl_param.kld_sampling.z_bin_width, double(0.2));
    amcl_ = std::make_shared<Amcl>(amcl_param);
    prediction_model_node_ = std::make_shared<FooPredictionModelNode>(amcl_);
  }
  // ros param
  pnh_.param<std::string>("world_frame_id", world_frame_id_, std::string("world"));
  // pnh_.param<std::string>("map_frame_id", map_frame_id_, std::string("map"));
  // pnh_.param<std::string>("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
  // pnh_.param<std::string>("odom_frame_id", odom_frame_id_, std::string("odom"));
  double publish_rate;
  pnh_.param<double>("publish_rate", publish_rate, double(100.0));
  pf_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles", 1, true);
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1, true);
  //  pc2_map_sub_ = nh_.subscribe("map", 10, &Amcl3dNode::mapCallback, this);
  init_pose_sub_ = nh_.subscribe("initialpose", 100, &Amcl3dNode::initialPoseCallback, this);
  ndt_pose_sub_ = nh_.subscribe("ndt_pose", 1, &Amcl3dNode::ndtPoseCallback, this);
  //  pc2_sub_ = nh_.subscribe("pc2", 1, &Amcl3dNode::pc2Callback, this);
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate), &Amcl3dNode::publishTimerCallback, this);
}





int main(int argc, char *argv[])
{
  ros::init(argc, argv, "amcl_3d");

  amcl_3d::Amcl3dNode node;
  ros::spin();

  return 0;
}