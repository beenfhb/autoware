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

AutowareMapSignalProjector::AutowareMapSignalProjector(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    roi_signal_pub_ = nh_.advertise<sensor_msgs::RegionOfInterest>("/target_roi_signal",1);
    pnh_.param<std::string>("proj_matrix_topic", proj_matrix_topic_, "/projection_matrix");
    pnh_.param<std::string>("camer_info_topic", camera_info_topic_, "/camera_info");
    autoware_map_.subscribe(nh_, autoware_map::Category::POINT);
    signal_light_sub_ = nh_.subscribe("/target_signal_light",1,&AutowareMapSignalProjector::targetSignalLightCallback,this);
    projection_matrix_sub_ = nh_.subscribe(proj_matrix_topic_,1,&AutowareMapSignalProjector::projectionMatrixCallback,this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_,1,&AutowareMapSignalProjector::cameraInfoCallback,this);
}

AutowareMapSignalProjector::~AutowareMapSignalProjector()
{

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
    Eigen::MatrixXd mat(4, 3);
    mat << msg->P[0],msg->P[1],msg->P[2],msg->P[3],msg->P[4],msg->P[4],msg->P[6],msg->P[7],msg->P[8],msg->P[9],msg->P[10],msg->P[11];
    p_matrix_ = mat;
    return;
}

void AutowareMapSignalProjector::targetSignalLightCallback(const autoware_map_msgs::SignalLightArray::ConstPtr msg)
{
    if((!proj_matrix_) || (!p_matrix_))
    {
        return;
    }
    sensor_msgs::RegionOfInterest roi_signal;
    //roi_signal.header = msg->header;
    for(auto itr = msg->data.begin(); itr != msg->data.end(); itr++)
    {
        autoware_map::Key<autoware_map_msgs::SignalLight> signal_key(itr->signal_light_id);
        autoware_map_msgs::SignalLight signal_light = autoware_map_.findByKey(signal_key);
        autoware_map::Key<autoware_map_msgs::Point> point_key(itr->point_id);
        autoware_map_msgs::Point point = autoware_map_.findByKey(point_key);
    }
    roi_signal_pub_.publish(roi_signal);
    return;
}