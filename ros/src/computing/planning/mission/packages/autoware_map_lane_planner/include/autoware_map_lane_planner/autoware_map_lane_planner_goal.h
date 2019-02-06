#ifndef AUTOWARE_MAP_LANE_PLANNER_GOAL_H_INCLUDED
#define AUTOWARE_MAP_LANE_PLANNER_GOAL_H_INCLUDED

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

//headers in boost
#include <boost/optional.hpp>

//headers in Autoware
#include <autoware_msgs/LaneArray.h>

//headers in ROS
#include <geometry_msgs/PoseStamped.h>

class AutowareMapLanePlannerGoal
{
public:
    AutowareMapLanePlannerGoal(){command_type_ = UNDEFINED;};
    ~AutowareMapLanePlannerGoal(){};
    enum COMMAND_TYPE{UNDEFINED = -1,POSE = 0,WAYPOINT = 1};
    int getCommandType(){return command_type_;};
    bool getCommand(geometry_msgs::PoseStamped& pose)
    {
        if(command_type_ == POSE && pose_cmd_)
        {
            pose = pose_cmd_.get();
            return true;
        }
        return false;
    };
    void setCommand(geometry_msgs::PoseStamped pose){pose_cmd_ = pose;}
    bool getCommand(autoware_msgs::LaneArray& waypoint)
    {
        if(command_type_ == WAYPOINT && waypoint_cmd_)
        {
            waypoint = waypoint_cmd_.get();
            return true;
        }
        return false;
    };
    void setCommand(autoware_msgs::LaneArray waypoint){waypoint_cmd_ = waypoint;}
private:
    int command_type_;
    boost::optional<geometry_msgs::PoseStamped> pose_cmd_;
    boost::optional<autoware_msgs::LaneArray> waypoint_cmd_;
};

#endif