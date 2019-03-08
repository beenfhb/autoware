#ifndef WAYPOINT_FOLLWER_PID_CONTROLLER_H_INCLUDED
#define WAYPOINT_FOLLWER_PID_CONTROLLER_H_INCLUDED
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

//headers in Boost
#include <boost/circular_buffer.hpp>

//headers in ROS
#include <ros/ros.h>

class PIDController
{
public:
    PIDController(double p_gain,double i_gain,double d_gain);
    ~PIDController();
    double getInput(double cmd,double value,ros::Time now);
    double getInput(double error,ros::Time now);
    const double p_gain;
    const double i_gain;
    const double d_gain;
private:
    boost::circular_buffer<std::pair<double,ros::Time> > errors_;
    double i_value_;
};

#endif  //WAYPOINT_FOLLWER_PID_CONTROLLER_H_INCLUDED