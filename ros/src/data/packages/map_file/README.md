# map_file package
## points_map_filter
### feature
points_map_filter_node subscribe pointcloud maps and current pose, the node extract pointcloud near to the current pose.

#### subscribed topics
/points_map (sensor_msgs/PointCloud2)  : Raw pointcloud map. This topic usually comes from points_map_loader.  
/current_pose (geometry_msgs/PoseStamped) : Current pose of the car. This topic usually comes from pose_relay node.  

#### published topics
/points_map/filtered (sensor_msgs/PointCloud2) : Filtered pointcloud submap.  

#### parameters
load_grid_size (double) : grid size of submap.  
load_trigger_distance (double) : if the car moves load_trigger_distance(m), the map filter publish filtered submap.

### how it works
map_filter_node relay /points_map topic until it recieves /current_pose topic.  
Then, the /current_pose topic recieved, the map_filter_node publish submap.

## demonstration
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/LpKIuI5b4DU/0.jpg)](http://www.youtube.com/watch?v=LpKIuI5b4DU)

## autoware_map_loader
### feature
autoware_map_loader loads semantic map information in Autoware Map Format.


#### subscribed topics
NONE

#### published topics
+ **awm_state** std_msgs::Bool  
  becomes true after the loading is finished
+ **autoware_map_info/lane** autoware_map_msgs::LaneArray    
+ **autoware_map_info/lane_attr_relation** autoware_map_msgs::LaneAttrRelationArray   
+ **autoware_map_info/lane_relation** autoware_map_msgs::LaneRelationArray   
+ **autoware_map_info/lane_signal_light_relation** autoware_map_msgs::LaneSignalLightRelationArray  
+ **autoware_map_info/lane_change_relation** autoware_map_msgs::LaneChangeRelationArray  
+ **autoware_map_info/opposite_lane_relation** autoware_map_msgs::OppositeLaneRelationArray     
+ **autoware_map_info/point** autoware_map_msgs::PointArray    
+ **autoware_map_info/area** autoware_map_msgs::AreaArray   
+ **autoware_map_info/route** autoware_map_msgs::RouteArray   
+ **autoware_map_info/signal** autoware_map_msgs::SignalArray   
+ **autoware_map_info/signal_light** autoware_map_msgs::SignalLightArray   
+ **autoware_map_info/wayarea** autoware_map_msgs::WayareaArray    
+ **autoware_map_info/waypoint** autoware_map_msgs::WaypointArray    
+ **autoware_map_info/waypoint_lane_relation** autoware_map_msgs::WaypointLaneRelationArray   
+ **autoware_map_info/waypoint_relation** autoware_map_msgs::WaypointRelationArray   
+ **autoware_map_info/waypoint_signal_relation** autoware_map_msgs::WaypointSignalRelationArray   

#### Usage
`rosrun map_file autoware_map_loader [FILE]`

#### Related Packages
+ **autoware_map_msgs package**: Defines the message types for Autoware Map Format
+ **autoware_map**: Provides code api to access to autoware_map_msgs.
