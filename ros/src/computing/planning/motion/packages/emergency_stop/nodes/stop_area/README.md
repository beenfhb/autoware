# Safety

Autoware package that area-based colision checker for emergency stop.

### Requirements

* Vehicle information(length, width, height, wheel-base...).

### How to launch

* From a sourced terminal:
    - `roslaunch points_preprocessor compare_map_filter.launch`

* From Runtime Manager:

Sensing Tab -> Points Preprocessor -> `compare_map_filter`
You can change the config, as well as other parameters, by clicking [app]

### Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`input_point_topic`|*String*|PointCloud source topic. Default `/points_raw`.|
|`baselink_frame`|*String*|base_link frame id. Default `/base_link`.|
|`sim_time`|*Double*|.  Default `1.0`.|
|`sim_time_delta`|*Double*|.  Default `1.0`.|
|`filtering_radius`|*Double*|.  Default `1.0`.|
|`filtering_points_size`|*Double*|.  Default `1.0`.|
|`offset_front`|*Double*|. Default `0.1`.|
|`offset_rear`|*Double*|. Default `0.1`.|
|`offset_left`|*Double*|. Default `0.1`.|
|`offset_right`|*Double*|. Default `0.1`.|
|`offset_top`|*Double*|. Default `0.1`.|
|`offset_bottom`|*Double*|. Default `-0.3`.|
|`use_vehicle_info_param`|*Bool*|. Default `TRUE`.|
|`vehicle_info_length`|*Double*|. Default `5.0`.|
|`vehicle_info_width`|*Double*|. Default `2.0`.|
|`vehicle_info_height`|*Double*|. Default `2.0`.|
|`vehicle_info_wheel_base`|*Double*|. Default `2.95`.|
|`vehicle_info_tread_front`|*Double*|. Default `1.5`.|
|`vehicle_info_tread_rear`|*Double*|. Default `1.5`.|
|`vehicle_info_center_to_base`|*Double*|. Default `-1.5`.|

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/points_raw`|`sensor_msgs/PointCloud2`|PointCloud source topic.|
|`/current_velocity`|`geometry_msgs/TwistStamped`|.|
|`/vehicle_cmd`|`autoware_msgs/VehicleCmd`|.|
|`/vehicle_status_sub_`|`autoware_msgs/VehicleStatus`|.|
|`/config/`|``|Configuration adjustment for threshold.|
|`/tf`|TF|sensor frame <-> base_link frame|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/points_obstacle`|`sensor_msgs/PointCloud2`|.|

### Video
