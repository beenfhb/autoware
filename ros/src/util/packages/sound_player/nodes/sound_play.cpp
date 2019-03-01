#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>

#include <thread>
#include <map>

std::map<std::string, std::string> map = {
  {"VehicleReady\nDriving\nDrive\nLaneArea\nCruise\nStraight\nGo\n", "/home/autoware/Downloads/voice/Go.ogg"},
  {"VehicleReady\nDriving\nDrive\nLaneArea\nCruise\nStraight\nStop\n", "/home/autoware/Downloads/voice/Stop.ogg"},
  {"VehicleReady\MissionComplete\WaitEngage\n", "/home/autoware/Downloads/voice/MissionComplete.ogg"},
};

void soundplayerCallback(const std_msgs::String::ConstPtr &msg)
{
  static std_msgs::String msg_;
  if(msg_.data == msg->data) {
    return;
  }
  msg_ = *msg;
    std::thread sound_play_thread([](const std_msgs::String &msg)
    {
      	std::cout << msg.data.c_str() << std::endl << std::endl;
        if(map.count(msg.data) != 0){
          sound_play::SoundClient sc;
          sc.playWave(map[msg.data]);
        	ros::Duration(3).sleep();
        	ros::spinOnce();
        }
    }, msg_);
    sound_play_thread.detach();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_player");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/decision_maker/state", 3, soundplayerCallback);

    sound_play::SoundClient sc;
    sc.playWave("/home/autoware/Downloads/output.ogg");
    ros::Duration(3).sleep();
    ros::spin();

    return 0;
}
