#ifndef __CROSSWALK_MANAGER_HPP__
#define __CROSSWALK_MANAGER_HPP__

#include "crosswalk.hpp"
#include "../velocity_set/libvelocity_set.h"

#include <autoware_map/autoware_map.h>
#include <autoware_msgs/Lane.h>

#include <std_msgs/UInt64.h>

class CrossWalkManager
{
private:

    // detection_points_[bdID] has information of each crosswalk
    std::map<int, CrossWalkHandler> crosswalks_;
    bool enable_multiple_crosswalk_detection_;
    std::vector<int> relevant_crosswalk_ids_;
    ros::NodeHandle nh;
    void addRelevantCrossWalkIDs(const int &id);
    void autowareMapCallback(const std_msgs::UInt64::ConstPtr &available_category);
    ros::Subscriber map_sub_;
    autoware_map::AutowareMap autoware_map_;

public:
    CrossWalkManager();
    void findRelevantCrossWalks(const int closest_waypoint, const autoware_msgs::Lane &lane, const int search_distance);
    std::vector<CrossWalkHandler> getRelevantCrossWalks(const int waypoint_id) const;
    std::vector<CrossWalkHandler> getRelevantCrossWalks() const;
    void setMultipleDetectionFlag(const bool _multiple_flag)
    {
        enable_multiple_crosswalk_detection_ = _multiple_flag;
    }
    bool useMultipleDetection() const
    {
        return enable_multiple_crosswalk_detection_;
    }
};

#endif
