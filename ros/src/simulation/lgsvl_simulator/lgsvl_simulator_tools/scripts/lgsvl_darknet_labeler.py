#!/usr/bin/env python
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from lgsvl_msgs.msg import Detection2DArray
import os
import datetime
import shutil
from cv_bridge import CvBridge, CvBridgeError

from labeled_image import LabeledImage

class LgsvlDarknetLabaler:
    def __init__(self):
        object_types = ["car","pedestrian","bicycle"]
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('lgsvl_simulator_tools')
        time_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.data_dir = self.package_path+"/"+time_str+"_data"
        os.mkdir(self.data_dir)
        os.mkdir(self.data_dir+"/data")
        self.datknet_path = rospack.get_path('vision_darknet_detect') + "/darknet"
        object_types_str = "" 
        i = 0
        self.labeled_images = {}
        for object_type in object_types:
            if i != (len(object_types)-1):
                object_types_str = object_types_str + object_type + "\n"
            else:
                object_types_str = object_types_str + object_type
            i = i + 1
            self.labeled_images[object_type] = LabeledImage(i-1,self.data_dir+"/data/"+str(i)+object_type)
            os.mkdir(self.data_dir+"/data/"+str(i)+object_type)
        with open(self.package_path+"/template/class/class.txt", mode='w') as f:
            f.write(object_types_str)
        with open(self.package_path+"/template/cfg/lgsvl.names", mode='w') as f:
            f.write(object_types_str)
        shutil.copytree(self.package_path+"/template/cfg", self.data_dir+"/cfg")
        shutil.copyfile(self.package_path+"/template/class/class.txt", self.data_dir+"/data/class.txt")
        self.image_sub = message_filters.Subscriber('/image_raw', Image)
        self.roi_sub = message_filters.Subscriber("/simulator/ground_truth/2d_detections", Detection2DArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.roi_sub], 10, 0.1, allow_headerless=True)
        self.count = 0
        self.sync.registerCallback(self.callback)
    def callback(self,image,roi):
        try:
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e
        for detection in roi.detections:
            self.labeled_images[detection.label].addROI(detection)
        for detection in roi.detections:
            self.labeled_images[detection.label].writeROIandImage(image)
        self.count = self.count + 1
    
        
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    labaler = LgsvlDarknetLabaler()
    rospy.spin()