# Overview
Local path planner for obstacle avoidance


# Input and Output
- input
    - /safety_waypoints : current waypoints
    - /current_pose : self pose
    - /current_velocity : self twist
    - /semantics/costmap_generator/occupancy_grid : occgrid for path selection
    
- output
    - /out_lane : local waypoints 




# Functions

Create multiple sub lanes translating the input waypoint in the horizontal direction. 
Calculate the cost of occgrid for each lanes, and publish the lane with the minimum cost as a waypoint.

# Parameter description

they are set in `lane_bypass_planner.config`

## for lane generation 

|Name|Type|Description|
|:---|:---|:---|
|enable_smooth_transition|bool|Generate lane from nearest waypoint with smooth curve|
|enable_smooth_transition_only_for_cost_calculation_and_vizualization|bool|Use smooth transition lane for cost calculation and vizualize, but publish non-smooth lane for lower layer. This is a temporary countermeasure against the deterioration of the trajectory tracking performance due to dynamically changing the lane shape.|
|enable_replan_when_moving|bool|When the center lane is selected, do not change the lane unless the vehicle speed below the stop speed|
|enable_force_lane_select|bool|An arbitrary path can be selected from the outside through topic|
|smooth_transit_dist|double|The length you want to make a transition smoothly|
|sub_lane_num|int|The number of sub lanes. This must be odd number (you can put even, but changed to odd)|
|sub_lane_width|double|width between sub lanes. |
|cost_check_num_max|int|maximum number of waypoints to check cost. This number times waypoints distance is the length for cost calculation.|


## for cost calculation

|Name|Type|Description|
|:---|:---|:---|
|cost_weight_be_center|double|The coefficient of cost which increase as going away from the central lane|
|cost_weight_stay_there|double| The coefficient of cost which increases as going away from the lane selected last time|
|cost_weight_stay_there_while|double|Same as "cost_weight_stay_there", but this value decreases to 0 as time progresses|
