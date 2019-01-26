#!/usr/bin/env python
import rospy
import rospkg
from lgsvl_msgs.msg import Detection2DArray
import os
import datetime
import shutil

class LgsvlDarknetLabaler:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('lgsvl_simulator_tools')
        time_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.data_dir = self.package_path+"/"+time_str+"_data"
        os.mkdir(self.data_dir)
        os.mkdir(self.data_dir+"/data")
        self.datknet_path = rospack.get_path('vision_darknet_detect') + "/darknet"
        object_types = "car\npedestrian\nbicycle"
        os.mkdir(self.data_dir+"/data/1car")
        os.mkdir(self.data_dir+"/data/2pedestrian")
        os.mkdir(self.data_dir+"/data/3bicycle")
        with open(self.package_path+"/template/class/class.txt", mode='w') as f:
            f.write(object_types)
        with open(self.package_path+"/template/cfg/lgsvl.names", mode='w') as f:
            f.write(object_types)
        shutil.copytree(self.package_path+"/template/cfg", self.data_dir+"/cfg")
        shutil.copyfile(self.package_path+"/template/class/class.txt", self.data_dir+"/data/class.txt")
        self.sub = rospy.Subscriber("/simulator/ground_truth/2d_detections", Detection2DArray, self.callback)
    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
        
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    labaler = LgsvlDarknetLabaler()
    rospy.spin()