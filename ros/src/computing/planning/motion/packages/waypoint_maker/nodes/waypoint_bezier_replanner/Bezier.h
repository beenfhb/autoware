#include "autoware_msgs/Lane.h"
#include "autoware_msgs/Waypoint.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <geometry_msgs/Pose.h>

namespace CubicBezier {
class CubicBezier {

public:
  CubicBezier();
  ~CubicBezier();

private:
  Vector3d start_point;
  Vector3d start_control_point;
  Vector3d end_control_point;
  Vector3d end_point;
  double getX(double t);
  double getY(double t);
  double getZ(double t);
  double getYaw(double t);
  VectorXd getPath();
  unsigned int resampling_num;
} CubicBezier(Vector3d &p1, Vector3d &p2, Vector3d &p3, Vecotr3d &p4, int num)
    : start_point(p1), start_control_point(p2), end_control_point(p3),
      end_point(p4), resampling_num(num){};
~CubicBezier();

double Bezier::getX(double t) {
  return (1 - t) * (1 - t) * (1 - t) * start_point[0] +
         3 * (1 - t) * (1 - t) * t * start_control_point[0] +
         3 * (1 - t) * t * t * end_control_point[0] +
         t * t * t * end_control_point[0];
}

double Bezier::getY(double t) {
  return (1 - t) * (1 - t) * (1 - t) * start_point[1] +
         3 * (1 - t) * (1 - t) * t * start_control_point[1] +
         3 * (1 - t) * t * t * end_control_point[1] +
         t * t * t * end_control_point[1];
}

double Bezier::getZ(double t) {
  return (1 - t) * (1 - t) * (1 - t) * start_point[2] +
         3 * (1 - t) * (1 - t) * t * start_control_point[2] +
         3 * (1 - t) * t * t * end_control_point[2] +
         t * t * t * end_control_point[2];
}

double Bezier::getYaw(double t) {
  dy = -3 * (1 - t) * (1 - t) * start_point[1] +
       3 * (1 - t) * (1 - 3 * t) * start_control_point[1] +
       3 * t * (2 - 3 * t) * end_control_point[1] + 3 * t * t * end[1];
  dx = -3 * (1 - t) * (1 - t) * start_point[0] +
       3 * (1 - t) * (1 - 3 * t) * start_control_point[0] +
       3 * t * (2 - 3 * t) * end_control_point[0] + 3 * t * t * end[0];
  return std::atan2(dy, dx);
}
VectorXd Bezier::getPath() {
  Eigen::VectorXd t_list = Eigen::VectorXd::LinSpaced(resampling_num, 0, 1);
  Eigen::MatrixXd path = Eigen::MatrixXd::Zero(resampling_num, 4);
  for (int i = 0; i < resampling_num; i++) {
    path(i, 0) = getX(t_list[i]);
    path(i, 1) = getY(t_list[i]);
    path(i, 2) = getZ(t_list[i]);
    path(i, 3) = getYaw(t_list[i]);
  }

  return path;
}
} // namespace

namespace waypoint_bezier_replanner {
autoware_msgs::Lane replan(const autoware_msgs::Lane &lane, const int step,
                           const int resampling_num) {
  autoware_msgs::Lane out_lane;
  int wp_size = lane.waypoints.size();
  int last_step = (wp_size - 1) % step;
  if (last_step == 0) {
    Eigen::MatrixXd p = Eigen::MatrixXd::Zero((wp_size - 1) / step + 1, 3);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero((wp_size - 1) / step + 1);
    Eigen::VectorXd change_flag =
        Eigen::VectorXd::Zero((wp_size - 1) / step + 1);
  } else {
    Eigen::MatrixXd p = Eigen::MatrixXd::Zero((wp_size - 1) / step + 2, 3);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero((wp_size - 1) / step + 2);
    Eigen::VectorXd change_flag =
        Eigen::VectorXd::Zero((wp_size - 1) / step + 2);
  }
  int idx_count = 0;
  int p_count = 0;
  for (auto i = lane.waypoints.begin(); i != lane.waypoints.end(); i++) {
    if (idx_count % step == 0 || i == lane.waypoints.end() - 1) {
      p(p_count, 0) = i->x;
      p(p_count, 1) = i->y;
      p(p_count, 2) = i->z;
      velocity(p_count) = i->velocity;
      change_flag(p_count) = i->change_flag;
      p_count++;
    }
    idx_count++;
  }
  Eigen::MatrixXd control_points = generateControlPoints(p);
  for (int i = 0; i < p_count-1; i++) {
    Vector3d p1(p(i, 0), p(i, 1), p(i, 2));
    Vector3d p2(control_points(2*i, 0), control_points(2*i, 1), control_points(2*i, 2));
    Vector3d p3(control_points(2*i+1, 0), control_points(2*i+1, 1), control_points(2*i+1, 2));
    Vector3d p4(p(i+1, 0), p(i+1, 1), p(i+1, 2));
    CubicBezier::CubicBezier bezier(p1, p2, p3, p4, resumpling_num);
    
  }
}

Eigen::MatrixXd generateControlPoints(const Eigen::MatrixXd &points) {
  int p_size = points.rows();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * (p_size - 1), 2 * (p_size - 1));
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2 * (p_size - 1), 3);
  A(0, 0) = -2;
  A(0, 1) = 1;
  A(2 * p_size - 3, 2 * p_size - 4) = 1;
  A[2 * p_size - 3, 2 * p_size - 3] = -2;
  b(0, 0) = -points(0, 0);
  b(0, 1) = -points(0, 1);
  b(0, 2) = -points(0, 2);
  b(2 * p_size - 3, 0) = -points(p_size - 1, 0);
  b(2 * p_size - 3, 1) = -points(p_size - 1, 1);
  b(2 * p_size - 3, 2) = -points(p_size - 1, 2);
  for (int i = 1; i < p_size - 1; i++) {
    A(i * 2 - 1, i * 2 - 1) = 1;
    A(i * 2 - 1, i * 2) = 1;
    A(i * 2, i * 2 - 2) = 1;
    A(i * 2, i * 2 - 1) = -2;
    A(i * 2, i * 2) = 2;
    A(i * 2, i * 2 + 1) = -1;
    b(i * 2 - 1, 0) = 2 * points(i, 0);
    b(i * 2, 0) = 0;
    b(i * 2 - 1, 1) = 2 * points(i, 1);
    b(i * 2, 1) = 0;
    b(i * 2 - 1, 2) = 2 * points(i, 2);
    b(i * 2, 2) = 0;
  }
  FullPivLU<MatrixXd> lu(A);
  x = lu.solve(b);
  return x;
}

} // namespace