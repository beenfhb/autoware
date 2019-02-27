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

#include "twist_gate.h"

TwistGate::TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_(nh)
  , seq_(0)
  , is_remote_(false)
  , is_remote_emergency_(false)
  , is_system_emergency_(false)
  , is_updated_(false)
  , is_state_drive_(true)
  , private_nh_(private_nh)
  , timeout_period_(1.0)
{
  node_status_pub_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);

  remote_cmd_sub_ = nh_.subscribe("/remote_cmd", 1, &TwistGate::remoteCmdCallback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::twistCmdCallback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] = nh_.subscribe("/mode_cmd", 1, &TwistGate::modeCmdCallback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] = nh_.subscribe("/gear_cmd", 1, &TwistGate::gearCmdCallback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] = nh_.subscribe("/accel_cmd", 1, &TwistGate::accelCmdCallback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] = nh_.subscribe("/steer_cmd", 1, &TwistGate::steerCmdCallback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] = nh_.subscribe("/brake_cmd", 1, &TwistGate::brakeCmdCallback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lampCmdCallback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrlCmdCallback, this);
  auto_cmd_sub_stdmap_["state"] = nh_.subscribe("/decision_maker/state", 1, &TwistGate::stateCallback, this);
  auto_cmd_sub_stdmap_["emergency_velocity"] =
      nh_.subscribe("/emergency_velocity", 1, &TwistGate::emergencyVelCallback, this);

  twist_gate_msg_.header.seq = 0;
  node_status_pub_ptr_->ENABLE();

  remote_cmd_time_ = emergency_time_ = ros::Time::now();
}

TwistGate::~TwistGate()
{
}

void TwistGate::run()
{
  node_status_pub_ptr_->NODE_ACTIVATE();
  ros::Rate r(100);
  for (; ros::ok(); r.sleep(), ++seq_)
  {
    is_updated_ = false;
    ros::spinOnce();
    if (!is_updated_)
    {
      continue;
    }

    if (isUsingDecisionmaker() && !is_state_drive_ && !is_remote_)
    {
      resetVehicleCmdMsg();
    }
    vehicle_cmd_pub_.publish(twist_gate_msg_);

    const bool is_error = (system_state_ == SystemState::ERROR);
    const bool is_fatal = (system_state_ == SystemState::FATAL);
    const ros::Time current = ros::Time::now();
    const ros::Duration remote_latency = current - remote_cmd_time_;
    const ros::Duration emergency_latency = current - emergency_time_;
    if ((is_error || is_fatal) && emergency_latency > timeout_period_ && !is_system_emergency_)
    {
      system_state_ = SystemState::NORMAL;
      twist_gate_msg_.emergency = 0;
    }
    if ((is_remote_ && remote_latency > timeout_period_) || is_remote_emergency_)
    {
      is_remote_ = false;
      is_remote_emergency_ = false;
    }
  }
}

void TwistGate::resetVehicleCmdMsg()
{
  twist_gate_msg_ = autoware_msgs::VehicleCmd();
  twist_gate_msg_.header.seq = seq_;
  twist_gate_msg_.ctrl_cmd.linear_velocity = -1;
}

bool TwistGate::isUsingDecisionmaker()
{
  bool using_decision_maker_flag = false;
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (const auto& i : node_list)
  {
    if (i == "/decision_maker")
    {
      using_decision_maker_flag = true;
      break;
    }
  }
  return using_decision_maker_flag;
}

void TwistGate::updateHeader(const std_msgs::Header& header)
{
  if ((header.stamp - twist_gate_msg_.header.stamp).toSec() > 0.0)
  {
    twist_gate_msg_.header.frame_id = header.frame_id;
    twist_gate_msg_.header.stamp = header.stamp;
  }
  twist_gate_msg_.header.seq = seq_;
}

void TwistGate::remoteCmdCallback(const remote_msgs_t::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  is_remote_ = (input_msg->control_mode == remote_msgs_t::MODE_REMOTE);

  remote_cmd_time_ = ros::Time::now();
  is_remote_emergency_ = static_cast<bool>(input_msg->vehicle_cmd.emergency);
  if (is_remote_ && system_state_ <= SystemState::ERROR)
  {
    std_msgs::Header header = twist_gate_msg_.header;
    twist_gate_msg_ = !is_remote_emergency_ ? input_msg->vehicle_cmd : vehicle_cmd_msg_t();
    twist_gate_msg_.header = header;
    updateHeader(input_msg->header);
  }
  twist_gate_msg_.emergency = input_msg->vehicle_cmd.emergency;
}

void TwistGate::twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  if (system_state_ <= SystemState::NORMAL && !is_remote_)
  {
    updateHeader(input_msg->header);
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;
  }
}

void TwistGate::modeCmdCallback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::NORMAL && !is_remote_)
  {
    // TODO:check this if statement
    if (input_msg->mode == -1 || input_msg->mode == 0)
    {
      resetVehicleCmdMsg();
    }
    updateHeader(input_msg->header);
    twist_gate_msg_.mode = input_msg->mode;
  }
}

void TwistGate::gearCmdCallback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::NORMAL && !is_remote_)
  {
    twist_gate_msg_.gear = input_msg->gear;
  }
}

void TwistGate::accelCmdCallback(const autoware_msgs::AccelCmd::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::DEBUG && !is_remote_)
  {
    updateHeader(input_msg->header);
    twist_gate_msg_.accel_cmd.accel = input_msg->accel;
  }
}

void TwistGate::steerCmdCallback(const autoware_msgs::SteerCmd::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::DEBUG && !is_remote_)
  {
    updateHeader(input_msg->header);
    twist_gate_msg_.steer_cmd.steer = input_msg->steer;
  }
}

void TwistGate::brakeCmdCallback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::DEBUG && !is_remote_)
  {
    updateHeader(input_msg->header);
    twist_gate_msg_.brake_cmd.brake = input_msg->brake;
  }
}

void TwistGate::lampCmdCallback(const autoware_msgs::LampCmd::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::NORMAL && !is_remote_)
  {
    updateHeader(input_msg->header);
    twist_gate_msg_.lamp_cmd.l = input_msg->l;
    twist_gate_msg_.lamp_cmd.r = input_msg->r;
  }
}

void TwistGate::ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  if (system_state_ <= SystemState::NORMAL && !is_remote_)
  {
    updateHeader(input_msg->header);
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;
  }
}

void TwistGate::stateCallback(const std_msgs::StringConstPtr& input_msg)
{
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  const bool is_drive = (input_msg->data.find("Drive\n") != std::string::npos);
  const bool is_vehicle_ready = (input_msg->data.find("VehicleReady\n") != std::string::npos);
  is_system_emergency_ = (input_msg->data.find("Emergency") != std::string::npos);
  is_state_drive_ = (is_drive && is_vehicle_ready);

  if (system_state_ <= SystemState::NORMAL && !is_remote_)
  {  // Set Parking Gear or Drive Gear
    const bool is_wait_order = (input_msg->data.find("WaitOrder") != std::string::npos);
    twist_gate_msg_.gear = (is_wait_order) ? CMD_GEAR_P : CMD_GEAR_D;
  }
}

void TwistGate::emergencyVelCallback(const vehicle_cmd_msg_t::ConstPtr& input_msg)
{
  is_updated_ = true;
  // TODO: in future, setup FATAL
  system_state_ = std::max(system_state_, SystemState::ERROR);
  emergency_time_ = ros::Time::now();
  if (system_state_ <= SystemState::ERROR && !is_remote_)
  {
    twist_gate_msg_ = *input_msg;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_gate");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  TwistGate twist_gate(nh, private_nh);
  twist_gate.run();
  return 0;
}
