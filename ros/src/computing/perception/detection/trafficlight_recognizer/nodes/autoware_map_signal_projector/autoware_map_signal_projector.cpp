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
    roi_signal_pub_ = nh_.advertise<autoware_msgs::Signals>("/roi_signal",1);
    signal_light_sub_ = nh_.subscribe("/target_signal_light",1,&AutowareMapSignalProjector::targetSignalLightCallback,this);
}

AutowareMapSignalProjector::~AutowareMapSignalProjector()
{

}

void AutowareMapSignalProjector::projectionMatrixCallback(const autoware_msgs::ProjectionMatrix::ConstPtr msg)
{
    proj_matrix_ = *msg;
    return;
}

void AutowareMapSignalProjector::targetSignalLightCallback(const autoware_map_msgs::SignalLightArray::ConstPtr msg)
{
    if(!proj_matrix_)
    {
        return;
    }
    autoware_msgs::Signals roi_signal;
    roi_signal.header = msg->header;
    for(auto itr = msg->data.begin(); itr != msg->data.end(); itr++)
    {

    }
    roi_signal_pub_.publish(roi_signal);
    return;
}