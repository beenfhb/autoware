#ifndef DRAW_LANE_H
#define DRAW_LANE_H
#include "autoware_msgs/ImageLaneObjects.h"
#include <opencv/cv.h>

namespace integrated_viewer {
// helper class to draw detected lane line
class DrawLane {
public:
  explicit DrawLane(void);
  void Draw(const autoware_msgs::ImageLaneObjects::ConstPtr &lane,
            cv::Mat &image);

protected:
  static const int kLineThickness;

private:
  static const cv::Scalar kRed;
};
} // namespace integrated_viewer
#endif // DRAW_LANE_H
