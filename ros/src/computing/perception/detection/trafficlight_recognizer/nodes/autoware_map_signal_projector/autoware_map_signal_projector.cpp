/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * ver 1.0 Masaya Kataoka
 */

//headers in Autoware
#include <autoware_map_signal_projector/autoware_map_signal_projector.h>

AutowareMapSignalProjector::AutowareMapSignalProjector(ros::NodeHandle nh,ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
{
    nh_ = nh;
    pnh_ = pnh;
    roi_signal_pub_ = nh_.advertise<autoware_msgs::SignalRoi>("/target_roi_signal",1);
    pnh_.param<std::string>("proj_matrix_topic", proj_matrix_topic_, "/projection_matrix");
    pnh_.param<std::string>("camer_info_topic", camera_info_topic_, "/camera_info");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<int>("signal_light_radius", signal_light_radius_, 10);
    autoware_map_.subscribe(nh_, autoware_map::Category::POINT);
    signal_light_sub_ = nh_.subscribe("/target_signal_light",1,&AutowareMapSignalProjector::targetSignalLightCallback,this);
    projection_matrix_sub_ = nh_.subscribe(proj_matrix_topic_,1,&AutowareMapSignalProjector::projectionMatrixCallback,this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_,1,&AutowareMapSignalProjector::cameraInfoCallback,this);
}

AutowareMapSignalProjector::~AutowareMapSignalProjector()
{

}

void AutowareMapSignalProjector::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr msg)
{
    if((!proj_matrix_) || (!p_matrix_) || (!target_roi_))
    {
        return;
    }
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(camera_frame_, map_frame_, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }
    autoware_msgs::SignalRoi roi_signal;
    roi_signal.header.stamp = msg->header.stamp;
    roi_signal.header.frame_id = camera_frame_;
    for(auto itr = target_roi_->data.begin(); itr != target_roi_->data.end(); itr++)
    {
        autoware_map::Key<autoware_map_msgs::SignalLight> signal_key(itr->signal_light_id);
        autoware_map_msgs::SignalLight signal_light = autoware_map_.findByKey(signal_key);
        autoware_map::Key<autoware_map_msgs::Point> point_key(signal_light.point_id);
        autoware_map_msgs::Point point = autoware_map_.findByKey(point_key);
        geometry_msgs::PointStamped point_stamped;
        point_stamped.header.frame_id = map_frame_;
        point_stamped.point.x = point.x;
        point_stamped.point.y = point.y;
        point_stamped.point.z = point.z;
        tf2::doTransform(point_stamped,point_stamped,transform_stamped);
        Eigen::MatrixXd point_mat(1, 4);
        point_mat << point_stamped.point.x,point_stamped.point.y,point_stamped.point.z,1;
        Eigen::MatrixXd point_transformed_mat(1,3);
        point_transformed_mat = p_matrix_.get() * proj_matrix_.get() * point_mat;
        autoware_msgs::SignalLightRoi roi_signal_light;
        roi_signal_light.signal_light_id = itr->signal_light_id;
        roi_signal_light.x = (int)point_transformed_mat(0,0);
        roi_signal_light.y = (int)point_transformed_mat(0,1);
        roi_signal_light.r = signal_light_radius_;
        roi_signal.signal_light_rois.push_back(roi_signal_light);
    }
    roi_signal_pub_.publish(roi_signal);
    return;
}

void AutowareMapSignalProjector::projectionMatrixCallback(const autoware_msgs::ProjectionMatrix::ConstPtr msg)
{
    Eigen::MatrixXd mat(4, 4);
    mat << msg->projection_matrix[0],msg->projection_matrix[1],msg->projection_matrix[2],msg->projection_matrix[3],
        msg->projection_matrix[4],msg->projection_matrix[4],msg->projection_matrix[6],msg->projection_matrix[7],
        msg->projection_matrix[8],msg->projection_matrix[9],msg->projection_matrix[10],msg->projection_matrix[11],
        msg->projection_matrix[12],msg->projection_matrix[13],msg->projection_matrix[14],msg->projection_matrix[15];
    proj_matrix_ = mat;
    return;
}

void AutowareMapSignalProjector::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr msg)
{
    camera_frame_ = msg->header.frame_id;
    Eigen::MatrixXd mat(4, 3);
    mat << msg->P[0],msg->P[1],msg->P[2],msg->P[3],msg->P[4],msg->P[4],msg->P[6],msg->P[7],msg->P[8],msg->P[9],msg->P[10],msg->P[11];
    p_matrix_ = mat;
    return;
}

void AutowareMapSignalProjector::targetSignalLightCallback(const autoware_map_msgs::SignalLightArray::ConstPtr msg)
{
    target_roi_ = *msg;
    return;
}