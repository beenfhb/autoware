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

// headers in this Autoware
#include <trajectory_following_controller/pid_controller.h>

PIDController::PIDController(double p_gain,double i_gain,double d_gain)
: p_gain(p_gain),i_gain(i_gain),d_gain(d_gain)
{
    errors_ = boost::circular_buffer<std::pair<double,ros::Time> >(2);
    i_value_ = 0;
}

PIDController::~PIDController()
{

}

double PIDController::getInput(double cmd,double value,ros::Time now)
{
    double error = value - cmd;
    i_value_ = i_value_ + error;
    double input = 0.0;
    double d_value = 0.0;
    errors_.push_back(std::make_pair(error,now));
    if(errors_.size() == 2)
    {
        d_value = (errors_[1].first-errors_[0].first)/(errors_[1].second-errors_[0].second).toSec();
    }
    input = error*p_gain + i_value_*i_gain + d_value*d_gain;
    return input;
}

double PIDController::getInput(double error,ros::Time now)
{
    i_value_ = i_value_ + error;
    double input = 0.0;
    double d_value = 0.0;
    errors_.push_back(std::make_pair(error,now));
    if(errors_.size() == 2)
    {
        d_value = (errors_[1].first-errors_[0].first)/(errors_[1].second-errors_[0].second).toSec();
    }
    input = error*p_gain + i_value_*i_gain + d_value*d_gain;
    return input;
}