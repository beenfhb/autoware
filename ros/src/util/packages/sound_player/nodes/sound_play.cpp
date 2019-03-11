#include <vector>
#include <map>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/LaneArray.h>

#include <sound_play/sound_play.h>

geometry_msgs::Point makePoint(const double x, const double y, const double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

struct BusStop {
    std::string name;
    geometry_msgs::Point point;
};

// struct PlaySoundTiming {
//     std::string sound_name;
//     geometry_msgs::Point point;
//     bool is_played;
// };

struct PlaySoundTiming {
    std::string start_bus_stop_name;
    std::string end_bus_stop_name;
    std::string sound_name;
    geometry_msgs::Point point;
    bool is_played;
};

std::map<std::string, std::string> sound_list = {
    {"Hello", "/home/autoware/Downloads/voice/Hello.ogg"},
    {"Go", "/home/autoware/Downloads/voice/Go.ogg"},
    {"Stop", "/home/autoware/Downloads/voice/Stop.ogg"},
    {"MissionComplete", "/home/autoware/Downloads/voice/MissionComplete.ogg"},
    {"StoppingSoon", "/home/autoware/Downloads/voice/StoppingSoon.ogg"},
    {"TrunRight", "/home/autoware/Downloads/voice/TrunRight.ogg"},
    {"TrunLeft", "/home/autoware/Downloads/voice/TrunLeft.ogg"},
    {"TryAvoid", "/home/autoware/Downloads/voice/TryAvoid.ogg"},
};

std::vector<BusStop> bus_stop_list = {
    {"A", makePoint(90309.2349, 94861.7154, 144.8269)},
    {"B", makePoint(90189.7578, 94759.1406, 139.9656)},
};

std::vector<PlaySoundTiming> play_sound_timing_list = {
    {"A", "B", "StoppingSoon", makePoint(90194.9531, 94767.3047, 140.0714), false},
    {"B", "A", "TrunRight", makePoint(90188.3047, 94757.6484, 139.9764), false}, //Uturn
    {"B", "A", "TrunRight", makePoint(90308.2266, 94866.7656, 144.8229), false}, //Uturn
    {"B", "A", "StoppingSoon", makePoint(90319.2969, 94866.5695, 144.8337), false},
};

std::map<std::string, std::string> state_sound_list = {
  {"VehicleReady\nDriving\nDrive\nLaneArea\nCruise\nStraight\nGo\n", "Go"},
  {"VehicleReady\nDriving\nDrive\nLaneArea\nCruise\nStraight\nStop\n", "Stop"},
  {"VehicleReady\nMissionComplete\nWaitEngage\n", "MissionComplete"},
};

void playSound(const std::string &sound_name)
{
    std::cout << "Play " << sound_name << std::endl;
    std::thread sound_play_thread([](const std::string &sound_name)
    {
      sound_play::SoundClient sc;
      sc.playWave(sound_list[sound_name]);
      ros::Duration(3).sleep();
      ros::spinOnce(); //TODO need?
    }, sound_name);
    sound_play_thread.detach();
}

double distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs)
{
    return std::sqrt(std::pow(lhs.x-rhs.x, 2.0)+std::pow(lhs.y-rhs.y, 2.0)+std::pow(lhs.z-rhs.z, 2.0));
}

std::string searchNearestBusStop(const geometry_msgs::Point& point)
{
    return std::min_element(std::begin(bus_stop_list), std::end(bus_stop_list),
        [point](const BusStop& lhs, const BusStop& rhs) {
            return distance(lhs.point, point) < distance(rhs.point, point);
        })->name;
}

std::string start_bus_stop_name;
std::string end_bus_stop_name;

void lanewaypointsCallback(const autoware_msgs::LaneArray::ConstPtr &msg)
{
    const auto start_bus_stop_position = msg->lanes.front().waypoints.front().pose.pose.position;
    const auto end_bus_stop_position = msg->lanes.front().waypoints.back().pose.pose.position;
    start_bus_stop_name = searchNearestBusStop(start_bus_stop_position);
    end_bus_stop_name = searchNearestBusStop(end_bus_stop_position);
    std::cout << start_bus_stop_name << " to " << end_bus_stop_name << std::endl;

    //TODO read config file

    //TODO make play_sound_timing_list
    for(auto& play_sound_timing : play_sound_timing_list) {
      play_sound_timing.is_played = false;
    }

    //TODO display

}

void currentposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    for(auto& play_sound_timing : play_sound_timing_list) {
        if(!play_sound_timing.is_played
        && play_sound_timing.start_bus_stop_name == start_bus_stop_name
        && play_sound_timing.end_bus_stop_name == end_bus_stop_name) {
            const double dis = distance(play_sound_timing.point, msg->pose.position);
            //TODO line over
            if(dis < 3.0) {
                play_sound_timing.is_played = true;
                playSound(play_sound_timing.sound_name);
            }
        }
    }
}

void stateCallback(const std_msgs::String::ConstPtr &msg)
{
  static std_msgs::String msg_;
  if(msg_.data == msg->data) {
    return;
  }

  msg_ = *msg;

  if(state_sound_list.count(msg_.data) != 0) {
      playSound(state_sound_list[msg_.data]);
  }

}

void soundnameCallback(const std_msgs::String::ConstPtr &msg)
{
  if(state_sound_list.count(msg_.data) != 0) {
    playSound(state_sound_list[msg_.data]);
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_player");

    ros::NodeHandle node;

    ros::Subscriber sub1 = node.subscribe("/based/lane_waypoints_raw", 1, lanewaypointsCallback);
    ros::Subscriber sub2 = node.subscribe("/current_pose", 1, currentposeCallback);
    ros::Subscriber sub3 = node.subscribe("/decision_maker/state", 3, stateCallback);
    ros::Subscriber sub4 = node.subscribe("/sound_name", 1, soundnamelCallback);

    sound_play::SoundClient sc;
    sc.playWave("/home/autoware/Downloads/Hello.ogg"); //dummy
    ros::Duration(3).sleep();
    ros::spin();

    return 0;
}
