# Astar Planner Package
This package contains three nodes for obstacle avoidance.

## Obstacle Avoid
## Velocity Set
Detects objects on waypoints and crosswalk and stops before collision.

## Autoware Map Velocity Set
Same functionality with normal velocity set, but uses autoware map format Implementation
Usage from runtime manager:
1. Load autoware map
2. Set up planners
3. Open app dialog for velocity_set
4. tick checkboxes for "Use Autoware Map", "Use Crosswalk Detection", and "Enable Multiple Crosswalk Detection". Click Ok.
5. start velocity set 
