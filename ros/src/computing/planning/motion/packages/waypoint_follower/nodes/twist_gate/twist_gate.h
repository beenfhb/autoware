#ifndef __TWIST_GATE_H__
#define __TWIST_GATE_H__
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

#include <chrono>
#include <iostream>
#include <map>
#include <thread>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/RemoteCmd.h"
#include "autoware_msgs/VehicleCmd.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "tablet_socket_msgs/gear_cmd.h"
#include "tablet_socket_msgs/mode_cmd.h"
// headers in Autowae Health Checker
#include <autoware_health_checker/health_checker/health_checker.h>

#define CMD_GEAR_D 1
#define CMD_GEAR_R 2
#define CMD_GEAR_B 3
#define CMD_GEAR_N 4
#define CMD_GEAR_P 5

enum SystemState
{
  DEBUG = 0,   // DEBUG is manually controlling mode(accel_cmd, mode_cmd, etc.)
  NORMAL = 1,  // NORMAL is normally automatic driving mode, and override DEBUG.
  ERROR = 2,   // In ERROR, the vehicle gently stops because of system trouble.
  FATAL = 3    // FATAL automatically stops because so dangerous that REMOTE is impossible.
};

class TwistGate
{
  using remote_msgs_t = autoware_msgs::RemoteCmd;
  using vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

  public:
    TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~TwistGate();
    void run();

  private:
    void updateHeader(const std_msgs::Header& header);
    bool isUsingDecisionmaker();
    void resetVehicleCmdMsg();

    void remoteCmdCallback(const remote_msgs_t::ConstPtr& input_msg);
    void twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg);
    void modeCmdCallback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg);
    void gearCmdCallback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg);
    void accelCmdCallback(const autoware_msgs::AccelCmd::ConstPtr& input_msg);
    void steerCmdCallback(const autoware_msgs::SteerCmd::ConstPtr& input_msg);
    void brakeCmdCallback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg);
    void lampCmdCallback(const autoware_msgs::LampCmd::ConstPtr& input_msg);
    void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg);
    void stateCallback(const std_msgs::StringConstPtr& input_msg);
    void emergencyVelCallback(const vehicle_cmd_msg_t::ConstPtr& input_msg);

    ros::NodeHandle nh_, private_nh_;
    std::shared_ptr<autoware_health_checker::HealthChecker> node_status_pub_ptr_;
    ros::Publisher vehicle_cmd_pub_;
    ros::Subscriber remote_cmd_sub_;
    std::map<std::string, ros::Subscriber> auto_cmd_sub_stdmap_;

    vehicle_cmd_msg_t twist_gate_msg_;
    bool is_remote_, is_remote_emergency_, is_system_emergency_;
    ros::Time remote_cmd_time_, emergency_time_;
    const ros::Duration timeout_period_;
    SystemState system_state_;
    std::thread watchdog_timer_thread_;

    int seq_;
    bool is_state_drive_, is_updated_;
};

#endif
