# Lidar Pose Estimation

The lane adjust node is modify the `pose.orientation` of input topic. Modify the direction of the object near the lane to fit the its direction. The others are overwriten to Z axis direction.

## Note

Currently, the nearest lane is found by linear search. That order is the product of lane points number and cluster number.

## Parameters

| Name | Type | Description | Default Value |
|------|------|-------------|---------------|
| object_topic_input  | String | Input topic.  | /detection/lidar_detector/objects |
| object_topic_output | String | Output topic. | /detection/object_filter/lane_adjust/objects |

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| The value of object_topic_input  | autoware_msgs::DetectedObjectArray | Input  | Detected object from a clustering algorithm or another object filter. |
| The value of object_topic_output | autoware_msgs::DetectedObjectArray | Output | Detected object that pose is modified to fit lane direction. |

