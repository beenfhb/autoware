/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
<<<<<<< HEAD
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>

#include "autoware_msgs/RemoteCmd.h"
#include "autoware_msgs/VehicleCmd.h"
#include "tablet_socket_msgs/mode_cmd.h"
#include "tablet_socket_msgs/gear_cmd.h"
#include "autoware_msgs/AccelCmd.h"
#include "autoware_msgs/BrakeCmd.h"
#include "autoware_msgs/SteerCmd.h"
#include "autoware_msgs/ControlCommandStamped.h"

//headers in Autowae Health Checker
#include <autoware_health_checker/health_checker/health_checker.h>

class TwistGate
{
  using remote_msgs_t = autoware_msgs::RemoteCmd;
  using vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

  public:
    TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~TwistGate();
  private:
    void watchdog_timer();
    void remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg);
    void auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg);
    void mode_cmd_callback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg);
    void gear_cmd_callback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg);
    void accel_cmd_callback(const autoware_msgs::AccelCmd::ConstPtr& input_msg);
    void steer_cmd_callback(const autoware_msgs::SteerCmd::ConstPtr& input_msg);
    void brake_cmd_callback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg);
    void lamp_cmd_callback(const autoware_msgs::LampCmd::ConstPtr& input_msg);
    void ctrl_cmd_callback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg);

    void reset_vehicle_cmd_msg();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<autoware_health_checker::HealthChecker> node_status_pub_ptr_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher control_command_pub_;
    ros::Publisher vehicle_cmd_pub_;
    ros::Publisher state_cmd_pub_;
    ros::Subscriber remote_cmd_sub_;
    std::map<std::string , ros::Subscriber> auto_cmd_sub_stdmap_;

    vehicle_cmd_msg_t twist_gate_msg_;
    std_msgs::Bool emergency_stop_msg_;
    ros::Time remote_cmd_time_;
    ros::Duration timeout_period_;

    std::thread watchdog_timer_thread_;
    enum class CommandMode{AUTO=1, REMOTE=2} command_mode_, previous_command_mode_;
    std_msgs::String command_mode_topic_;

    // still send is true
    bool send_emergency_cmd = false;
};

TwistGate::TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
     nh_(nh)
    ,private_nh_(private_nh)
    ,timeout_period_(1.0)
    ,command_mode_(CommandMode::AUTO)
    ,previous_command_mode_(CommandMode::AUTO)
{
  node_status_pub_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_,private_nh_);
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1, true);
  control_command_pub_ = nh_.advertise<std_msgs::String>("/ctrl_mode", 1);
=======
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "twist_gate.h"

TwistGate::TwistGate(const ros::NodeHandle &nh,
                     const ros::NodeHandle &private_nh)
    : nh_(nh), seq_(0), is_remote_(false), is_remote_emergency_(false),
      is_system_emergency_(false), is_updated_(false), is_state_drive_(true),
      private_nh_(private_nh), timeout_period_(1.0) {
  health_checker_ptr_ =
      std::make_shared<autoware_health_checker::HealthChecker>(nh_,
                                                               private_nh_);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);
  state_cmd_pub_ = nh_.advertise<std_msgs::Int32>("/state_cmd", 1, true);

<<<<<<< HEAD
  remote_cmd_sub_ = nh_.subscribe("/remote_cmd", 1, &TwistGate::remote_cmd_callback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::auto_cmd_twist_cmd_callback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] = nh_.subscribe("/mode_cmd", 1, &TwistGate::mode_cmd_callback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] = nh_.subscribe("/gear_cmd", 1, &TwistGate::gear_cmd_callback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] = nh_.subscribe("/accel_cmd", 1, &TwistGate::accel_cmd_callback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] = nh_.subscribe("/steer_cmd", 1, &TwistGate::steer_cmd_callback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] = nh_.subscribe("/brake_cmd", 1, &TwistGate::brake_cmd_callback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lamp_cmd_callback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrl_cmd_callback, this);

  twist_gate_msg_.header.seq = 0;
  emergency_stop_msg_.data = false;
  send_emergency_cmd = false;
  node_status_pub_ptr_->ENABLE();
=======
  remote_cmd_sub_ =
      nh_.subscribe("/remote_cmd", 1, &TwistGate::remoteCmdCallback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] =
      nh_.subscribe("/twist_cmd", 1, &TwistGate::twistCmdCallback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] =
      nh_.subscribe("/mode_cmd", 1, &TwistGate::modeCmdCallback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] =
      nh_.subscribe("/gear_cmd", 1, &TwistGate::gearCmdCallback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] =
      nh_.subscribe("/accel_cmd", 1, &TwistGate::accelCmdCallback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] =
      nh_.subscribe("/steer_cmd", 1, &TwistGate::steerCmdCallback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] =
      nh_.subscribe("/brake_cmd", 1, &TwistGate::brakeCmdCallback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] =
      nh_.subscribe("/lamp_cmd", 1, &TwistGate::lampCmdCallback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] =
      nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrlCmdCallback, this);
  auto_cmd_sub_stdmap_["state"] = nh_.subscribe(
      "/decision_maker/state", 1, &TwistGate::stateCallback, this);
  auto_cmd_sub_stdmap_["emergency_velocity"] = nh_.subscribe(
      "/emergency_velocity", 1, &TwistGate::emergencyVelCallback, this);

  twist_gate_msg_.header.seq = 0;
  health_checker_ptr_->ENABLE();
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format

  remote_cmd_time_ = ros::Time::now();
  watchdog_timer_thread_ = std::thread(&TwistGate::watchdog_timer, this);
  watchdog_timer_thread_.detach();
}

TwistGate::~TwistGate() {}

<<<<<<< HEAD
void TwistGate::reset_vehicle_cmd_msg()
{
  twist_gate_msg_.twist_cmd.twist.linear.x  = 0;
  twist_gate_msg_.twist_cmd.twist.angular.z = 0;
  twist_gate_msg_.mode                      = 0;
  twist_gate_msg_.gear                      = 0;
  twist_gate_msg_.lamp_cmd.l                = 0;
  twist_gate_msg_.lamp_cmd.r                = 0;
  twist_gate_msg_.accel_cmd.accel           = 0;
  twist_gate_msg_.brake_cmd.brake           = 0;
  twist_gate_msg_.steer_cmd.steer           = 0;
  twist_gate_msg_.ctrl_cmd.linear_velocity  = -1;
  twist_gate_msg_.ctrl_cmd.steering_angle   = 0;
}

void TwistGate::watchdog_timer()
{
  while(1)
  {
    ros::Time now_time = ros::Time::now();
    bool emergency_flag = false;

    // check command mode
    if(previous_command_mode_ != command_mode_) {
      if(command_mode_ == CommandMode::AUTO) {
        command_mode_topic_.data = "AUTO";
      }
      else if(command_mode_ == CommandMode::REMOTE) {
        command_mode_topic_.data = "REMOTE";
      }
      else{
        command_mode_topic_.data = "UNDEFINED";
      }

      control_command_pub_.publish(command_mode_topic_);
      previous_command_mode_ = command_mode_;
=======
void TwistGate::run() {
  health_checker_ptr_->NODE_ACTIVATE();
  ros::Rate r(100);
  for (; ros::ok(); r.sleep(), ++seq_) {
    is_updated_ = false;
    ros::spinOnce();
    if (!is_updated_) {
      continue;
    }

    if (isUsingDecisionmaker() && !is_state_drive_ && !is_remote_) {
      resetVehicleCmdMsg();
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    }

<<<<<<< HEAD
    // if lost Communication
    if(command_mode_ == CommandMode::REMOTE && now_time - remote_cmd_time_ >  timeout_period_) {
      emergency_flag = true;
      ROS_WARN("Lost Communication!");
    }

    // if push emergency stop button
    if(emergency_stop_msg_.data == true)
    {
      emergency_flag = true;
      ROS_WARN("Emergency Mode!");
    }

    // Emergency
    if(emergency_flag) {
      // Change Auto Mode
      command_mode_ = CommandMode::AUTO;
      if(send_emergency_cmd == false) {
        // Change State to Stop
        std_msgs::Int32 state_cmd;
        state_cmd.data = 14;
        state_cmd_pub_.publish(state_cmd);
        send_emergency_cmd = true;
      }
      // Set Emergency Stop
      emergency_stop_msg_.data = true;
      emergency_stop_pub_.publish(emergency_stop_msg_);
      ROS_WARN("Emergency Stop!");
=======
    const bool is_error = (system_state_ == SystemState::ERROR);
    const bool is_fatal = (system_state_ == SystemState::FATAL);
    const ros::Time current = ros::Time::now();
    const ros::Duration remote_latency = current - remote_cmd_time_;
    const ros::Duration emergency_latency = current - emergency_time_;
    if ((is_error || is_fatal) && emergency_latency > timeout_period_ &&
        !is_system_emergency_) {
      system_state_ = SystemState::NORMAL;
      twist_gate_msg_.emergency = 0;
    }
    if ((is_remote_ && remote_latency > timeout_period_) ||
        is_remote_emergency_) {
      is_remote_ = false;
      is_remote_emergency_ = false;
    }
  }
}

void TwistGate::resetVehicleCmdMsg() {
  twist_gate_msg_ = autoware_msgs::VehicleCmd();
  twist_gate_msg_.header.seq = seq_;
  twist_gate_msg_.ctrl_cmd.linear_velocity = -1;
}

bool TwistGate::isUsingDecisionmaker() {
  bool using_decision_maker_flag = false;
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (const auto &i : node_list) {
    if (i == "/decision_maker") {
      using_decision_maker_flag = true;
      break;
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    }

<<<<<<< HEAD
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
=======
void TwistGate::updateHeader(const std_msgs::Header &header) {
  if ((header.stamp - twist_gate_msg_.header.stamp).toSec() > 0.0) {
    twist_gate_msg_.header.frame_id = header.frame_id;
    twist_gate_msg_.header.stamp = header.stamp;
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
  }
}

<<<<<<< HEAD
void TwistGate::remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg)
{
  command_mode_ = static_cast<CommandMode>(input_msg->control_mode);
  emergency_stop_msg_.data = static_cast<bool>(input_msg->vehicle_cmd.emergency);
  remote_cmd_time_ = ros::Time::now();

  if(command_mode_ == CommandMode::REMOTE)
  {
    twist_gate_msg_.header.frame_id = input_msg->vehicle_cmd.header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->vehicle_cmd.header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->vehicle_cmd.twist_cmd.twist;
    twist_gate_msg_.ctrl_cmd  = input_msg->vehicle_cmd.ctrl_cmd;
    twist_gate_msg_.accel_cmd = input_msg->vehicle_cmd.accel_cmd;
    twist_gate_msg_.brake_cmd = input_msg->vehicle_cmd.brake_cmd;
    twist_gate_msg_.steer_cmd = input_msg->vehicle_cmd.steer_cmd;
    twist_gate_msg_.gear = input_msg->vehicle_cmd.gear;
    twist_gate_msg_.lamp_cmd = input_msg->vehicle_cmd.lamp_cmd;
    twist_gate_msg_.mode = input_msg->vehicle_cmd.mode;
    twist_gate_msg_.emergency = input_msg->vehicle_cmd.emergency;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
=======
void TwistGate::remoteCmdCallback(const remote_msgs_t::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  is_remote_ = (input_msg->control_mode == remote_msgs_t::MODE_REMOTE);

  remote_cmd_time_ = ros::Time::now();
  is_remote_emergency_ = static_cast<bool>(input_msg->vehicle_cmd.emergency);
  if (is_remote_ && system_state_ <= SystemState::ERROR) {
    std_msgs::Header header = twist_gate_msg_.header;
    twist_gate_msg_ =
        !is_remote_emergency_ ? input_msg->vehicle_cmd : vehicle_cmd_msg_t();
    twist_gate_msg_.header = header;
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
  }
}

<<<<<<< HEAD
void TwistGate::auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  node_status_pub_ptr_->NODE_ACTIVATE();
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
=======
void TwistGate::twistCmdCallback(
    const geometry_msgs::TwistStamped::ConstPtr &input_msg) {
  health_checker_ptr_->CHECK_RATE(
      "topic_twist_cmd_subscribe_rate_slow_in_twist_gate", 8.0, 4.0, 2.0,
      "topic twist_cmd subscribe rate is slow in twist_gate");
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  if (system_state_ <= SystemState::NORMAL && !is_remote_) {
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::mode_cmd_callback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    //TODO:check this if statement
    if(input_msg->mode == -1 || input_msg->mode == 0){
      reset_vehicle_cmd_msg();
=======
void TwistGate::modeCmdCallback(
    const tablet_socket_msgs::mode_cmd::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::NORMAL && !is_remote_) {
    // TODO:check this if statement
    if (input_msg->mode == -1 || input_msg->mode == 0) {
      resetVehicleCmdMsg();
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    }
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.mode = input_msg->mode;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::gear_cmd_callback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
=======
void TwistGate::gearCmdCallback(
    const tablet_socket_msgs::gear_cmd::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::NORMAL && !is_remote_) {
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.gear = input_msg->gear;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::accel_cmd_callback(const autoware_msgs::AccelCmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
=======
void TwistGate::accelCmdCallback(
    const autoware_msgs::AccelCmd::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::DEBUG && !is_remote_) {
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.accel_cmd.accel = input_msg->accel;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::steer_cmd_callback(const autoware_msgs::SteerCmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
=======
void TwistGate::steerCmdCallback(
    const autoware_msgs::SteerCmd::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::DEBUG && !is_remote_) {
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.steer_cmd.steer = input_msg->steer;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::brake_cmd_callback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
=======
void TwistGate::brakeCmdCallback(
    const autoware_msgs::BrakeCmd::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::DEBUG && !is_remote_) {
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.brake_cmd.brake = input_msg->brake;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::lamp_cmd_callback(const autoware_msgs::LampCmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
=======
void TwistGate::lampCmdCallback(
    const autoware_msgs::LampCmd::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::DEBUG);
  if (system_state_ <= SystemState::NORMAL && !is_remote_) {
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.lamp_cmd.l = input_msg->l;
    twist_gate_msg_.lamp_cmd.r = input_msg->r;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
void TwistGate::ctrl_cmd_callback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
=======
void TwistGate::ctrlCmdCallback(
    const autoware_msgs::ControlCommandStamped::ConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  if (system_state_ <= SystemState::NORMAL && !is_remote_) {
    updateHeader(input_msg->header);
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

<<<<<<< HEAD
=======
void TwistGate::stateCallback(const std_msgs::StringConstPtr &input_msg) {
  is_updated_ = true;
  system_state_ = std::max(system_state_, SystemState::NORMAL);
  const bool is_drive = (input_msg->data.find("Drive\n") != std::string::npos);
  const bool is_vehicle_ready =
      (input_msg->data.find("VehicleReady\n") != std::string::npos);
  is_system_emergency_ =
      (input_msg->data.find("Emergency") != std::string::npos);
  is_state_drive_ = (is_drive && is_vehicle_ready);

  if (system_state_ <= SystemState::NORMAL &&
      !is_remote_) { // Set Parking Gear or Drive Gear
    const bool is_wait_order =
        (input_msg->data.find("WaitOrder") != std::string::npos);
    twist_gate_msg_.gear = (is_wait_order) ? CMD_GEAR_P : CMD_GEAR_D;
  }
}

void TwistGate::emergencyVelCallback(
    const vehicle_cmd_msg_t::ConstPtr &input_msg) {
  is_updated_ = true;
  // TODO: in future, setup FATAL
  system_state_ = std::max(system_state_, SystemState::ERROR);
  emergency_time_ = ros::Time::now();
  if (system_state_ <= SystemState::ERROR && !is_remote_) {
    twist_gate_msg_ = *input_msg;
  }
}
>>>>>>> de1af71... add CHECK_RATE function and apply clang-format

int main(int argc, char **argv) {
  ros::init(argc, argv, "twist_gate");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  TwistGate twist_gate(nh, private_nh);

  ros::spin();
  return 0;
}
