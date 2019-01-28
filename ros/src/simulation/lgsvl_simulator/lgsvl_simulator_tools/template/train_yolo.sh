wget https://pjreddie.com/media/files/darknet53.conv.74
rosrun vision_darknet_detect darknet detector train cfg/lgsvl.data cfg/yolov3.cfg darknet53.conv.74