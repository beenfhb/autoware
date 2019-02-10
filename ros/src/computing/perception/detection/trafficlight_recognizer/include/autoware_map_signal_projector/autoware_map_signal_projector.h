#ifndef AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED
#define AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED

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

//headers in ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Int32.h>

class AutowareMapSignalProjector
{
public:
    AutowareMapSignalProjector(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AutowareMapSignalProjector();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void targetSignalIndexCallback(const std_msgs::Int32::ConstPtr msg);
};

#endif  //AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED