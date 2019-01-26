#!/usr/bin/env python
import rospy
import rospkg
from lgsvl_msgs.msg import Detection2DArray
import os
import datetime

class LgsvlDarknetLabaler:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('lgsvl_simulator_tools')
        time_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        os.mkdir(self.package_path+"/"+time_str+"_data")
        self.datknet_path = rospack.get_path('vision_darknet_detect') + "/darknet"
        object_types = "car\n"
        self.sub = rospy.Subscriber("/simulator/ground_truth/2d_detections", Detection2DArray, self.callback)
    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
        
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    labaler = LgsvlDarknetLabaler()
    rospy.spin()