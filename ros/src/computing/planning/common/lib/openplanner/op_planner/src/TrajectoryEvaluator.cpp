/// \file TrajectoryEvaluator.cpp
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local planner version 1.5+ (Dynamic Obstacles Prediction)
/// \author Hatem Darweesh
/// \date Jan 3, 2019

#include "op_planner/TrajectoryEvaluator.h"
#include "op_planner/MatrixOperations.h"
#include "float.h"

namespace PlannerHNS
{

TrajectoryEvaluator::TrajectoryEvaluator()
{
}

TrajectoryEvaluator::~TrajectoryEvaluator()
{
}

TrajectoryCost TrajectoryEvaluator::doOneStep(const std::vector<std::vector<WayPoint> >& roll_outs,
                                              const std::vector<WayPoint>& total_paths, const WayPoint& curr_state,
                                              const PlanningParams& params, const CAR_BASIC_INFO& car_info,
                                              const VehicleState& vehicle_state,
                                              const std::vector<DetectedObject>& obj_list,
                                              const bool& b_static_only,
                                              const int& prev_curr_index)
{

  timespec _t;
  op_utility_ns::UtilityH::GetTickCount(_t);

  double critical_lateral_distance = car_info.width / 2.0 + params.horizontalSafetyDistancel;
  double critical_long_front_distance = car_info.wheel_base / 2.0 + car_info.length / 2.0
      + params.verticalSafetyDistance;
  double critical_long_back_distance = car_info.length / 2.0 + params.verticalSafetyDistance
      - car_info.wheel_base / 2.0;

  int curr_index = -1;
  if (curr_index >= 0 && curr_index < roll_outs.size())
    curr_index = prev_curr_index;
  else
    curr_index = getCurrentRollOutIndex(total_paths, curr_state, params);

  initializeLocalRollOuts(curr_state, car_info, params, critical_long_back_distance, roll_outs, local_roll_outs_);

  initializeSafetyPolygon(curr_state, car_info, vehicle_state, critical_lateral_distance, critical_long_front_distance,
                            critical_long_back_distance, safety_border_);

  initializeCosts(roll_outs, params, trajectory_costs_);

  calculateTransitionCosts(trajectory_costs_, curr_index, params);

  collectContoursAndTrajectories(obj_list, all_contour_points_, all_trajectories_points_, b_static_only);

  collision_points_.clear();
  calculateDistanceCosts(params, critical_lateral_distance, local_roll_outs_, all_contour_points_, all_trajectories_points_, trajectory_costs_, collision_points_);
//  collision_points_.clear();
//  collision_points_.insert(collision_points_.begin(), all_contour_points_.begin(), all_contour_points_.end());
//  collision_points_.insert(collision_points_.begin(), all_trajectories_points_.begin(), all_trajectories_points_.end());

  normalizeCosts(trajectory_costs_, weights_);

  TrajectoryCost best_trajectory = findBestTrajectory(params, trajectory_costs_);

//	cout << "Smallest Index: " << smallestIndex <<", Costs Size: " << trajectory_costs_.size() << ", Rollout Index: " << bestTrajectory.index << ", Closest Distance: " << bestTrajectory.closest_obj_distance << endl;
//	cout << "------------------------------------------------------------- " << endl;

//  double dt = op_utility_ns::UtilityH::GetTimeDiffNow(_t);
//  std::cout << "Contour points: " << all_contour_points_.size() << ", Trajectory Points: "
//        << all_trajectories_points_.size() << ", dt: " << dt <<  std::endl;
  return best_trajectory;
}

void TrajectoryEvaluator::initializeLocalRollOuts(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info, const PlanningParams& params, const double& c_long_back_d, const std::vector<std::vector<WayPoint> >& original_roll_outs, std::vector<std::vector<WayPoint> >& local_roll_outs)
{
  local_roll_outs = original_roll_outs;
  int center_index = params.rollOutNumber / 2;

  for(unsigned int i=0; i < local_roll_outs.size(); i++)
  {
    //std::cout << "RollOut: " << i << ", Size: " <<  local_roll_outs.at(i).size() << std::endl;
    if(local_roll_outs.at(i).size() > 0)
    {
        WayPoint center_back_point = local_roll_outs.at(i).at(0);
        center_back_point.pos.x -= c_long_back_d * cos(curr_state.pos.a);
        center_back_point.pos.y -= c_long_back_d * sin(curr_state.pos.a);
        local_roll_outs.at(i).insert(local_roll_outs.at(i).begin(), center_back_point);
        PlannerHNS::PlanningHelpers::CalcAngleAndCost(local_roll_outs.at(i));
    }
  }

  for(unsigned int i=0; i < local_roll_outs.size(); i++)
  {
    for(unsigned int j=0; j < local_roll_outs.at(i).size(); j++)
    {
      if(local_roll_outs.at(center_index).size() > j)
        local_roll_outs.at(i).at(j).width = hypot(local_roll_outs.at(center_index).at(j).pos.y - local_roll_outs.at(i).at(j).pos.y, local_roll_outs.at(center_index).at(j).pos.x - local_roll_outs.at(i).at(j).pos.x);
      else
        std::cout << "Error .. RollOuts are not synchronized , check TrajectoryEvaluator, initializeLocalRollOuts (center_size, j) (" << local_roll_outs.at(center_index).size() <<", " << j <<")"  << std::endl;
    }
  }
}

void TrajectoryEvaluator::collectContoursAndTrajectories(const std::vector<PlannerHNS::DetectedObject>& obj_list,
                                                         std::vector<WayPoint>& contour_points,
                                                         std::vector<WayPoint>& trajectory_points,
                                                         const bool& b_static_only)
{
  WayPoint p;
  double d = 0;
  contour_points.clear();
  trajectory_points.clear();
  for (unsigned int i = 0; i < obj_list.size(); i++)
  {
    double w = obj_list.at(i).w / 2.0;

    for (unsigned int i_con = 0; i_con < obj_list.at(i).contour.size(); i_con++)
    {
      p.pos = obj_list.at(i).contour.at(i_con);
      p.pos.a = obj_list.at(i).center.pos.a;
      p.v = obj_list.at(i).center.v;
      p.id = i;
      p.width = 0;
      contour_points.push_back(p);
    }

    if(b_static_only)
    {
      continue;
    }
//      if(obj_list.at(i).bVelocity)
//        continue;

    for (unsigned int i_trj = 0; i_trj < obj_list.at(i).predTrajectories.size(); i_trj++)
    {
      for (unsigned int i_p = 0; i_p < obj_list.at(i).predTrajectories.at(i_trj).size(); i_p++)
      {
        p = obj_list.at(i).predTrajectories.at(i_trj).at(i_p);

        if(hypot(obj_list.at(i).center.pos.y-p.pos.y, obj_list.at(i).center.pos.x-p.pos.x) < obj_list.at(i).l/2.0)
        {
          continue;
        }

        p.id = i;
        p.width = w;

        bool b_blocking = false;
        for(unsigned int k=0; k < obj_list.size(); k++)
        {
          if(i != k && PlanningHelpers::PointInsidePolygon(obj_list.at(k).contour, p.pos) == 1)
          {
            b_blocking = true;
            break;
          }
        }

        if(b_blocking)
        {
          break;
        }

        bool b_found_point = false;
        for(unsigned int k=0; k < trajectory_points.size(); k++)
        {
          if(hypot(trajectory_points.at(k).pos.y - p.pos.y, trajectory_points.at(k).pos.x - p.pos.x) < 0.25)
          {
            if(p.width > trajectory_points.at(k).width)
            {
              trajectory_points.at(k).width = p.width;
            }

            b_found_point = true;
            break;
          }
        }

        if(!b_found_point)
          trajectory_points.push_back(p);
      }
    }
  }
}

void TrajectoryEvaluator::normalizeCosts(std::vector<TrajectoryCost>& trajectory_costs,
                                         const EvaluationWeights& weights)
{
  double total_priorities = 0;
  double total_change = 0;
  double total_lateral_costs = 0;
  double total_longitudinal_costs = 0;
  double transition_costs = 0;

  for (unsigned int ic = 0; ic < trajectory_costs.size(); ic++)
  {
    total_priorities += trajectory_costs.at(ic).priority_cost;
    transition_costs += trajectory_costs.at(ic).transition_cost;
  }

  for (unsigned int ic = 0; ic < trajectory_costs.size(); ic++)
  {
    total_change += trajectory_costs.at(ic).lane_change_cost;
    total_lateral_costs += trajectory_costs.at(ic).lateral_cost;
    total_longitudinal_costs += trajectory_costs.at(ic).longitudinal_cost;
  }

//	cout << "------ Normalizing Step " << endl;
  for (unsigned int ic = 0; ic < trajectory_costs.size(); ic++)
  {
    if (total_priorities != 0)
      trajectory_costs.at(ic).priority_cost = trajectory_costs.at(ic).priority_cost / total_priorities;
    else
      trajectory_costs.at(ic).priority_cost = 0;

    if (transition_costs != 0)
      trajectory_costs.at(ic).transition_cost = trajectory_costs.at(ic).transition_cost / transition_costs;
    else
      trajectory_costs.at(ic).transition_cost = 0;

    if (total_change != 0)
      trajectory_costs.at(ic).lane_change_cost = trajectory_costs.at(ic).lane_change_cost / total_change;
    else
      trajectory_costs.at(ic).lane_change_cost = 0;

    if (total_lateral_costs != 0)
      trajectory_costs.at(ic).lateral_cost = trajectory_costs.at(ic).lateral_cost / total_lateral_costs;
    else
      trajectory_costs.at(ic).lateral_cost = 0;

    if (total_longitudinal_costs != 0)
      trajectory_costs.at(ic).longitudinal_cost = trajectory_costs.at(ic).longitudinal_cost / total_longitudinal_costs;
    else
      trajectory_costs.at(ic).longitudinal_cost = 0;

    trajectory_costs.at(ic).cost = (weights.priority_weight_ * trajectory_costs.at(ic).priority_cost
        + weights.transition_weight_ * trajectory_costs.at(ic).transition_cost
        + weights.lateral_weight_ * trajectory_costs.at(ic).lateral_cost
        + weights.longitudinal_weight_ * trajectory_costs.at(ic).longitudinal_cost);

//		cout << "Index: " << ic
//						<< ", Priority: " << trajectory_costs.at(ic).priority_cost
//						<< ", Transition: " << trajectory_costs.at(ic).transition_cost
//						<< ", Lat: " << trajectory_costs.at(ic).lateral_cost
//						<< ", Long: " << trajectory_costs.at(ic).longitudinal_cost
//						<< ", Change: " << trajectory_costs.at(ic).lane_change_cost
//						<< ", Avg: " << trajectory_costs.at(ic).cost
//						<< endl;
  }

//	cout << "------------------------ " << endl;
}

void TrajectoryEvaluator::calculateTransitionCosts(std::vector<TrajectoryCost>& trajectory_costs,
                                                   const int& curr_index, const PlanningParams& params)
{
  for (int ic = 0; ic < trajectory_costs.size(); ic++)
  {
    trajectory_costs.at(ic).transition_cost = fabs(params.rollOutDensity * (ic - curr_index));
  }
}

int TrajectoryEvaluator::getCurrentRollOutIndex(const std::vector<WayPoint>& path, const WayPoint& curr_state,
                                                const PlanningParams& params)
{
  RelativeInfo obj_info;
  PlanningHelpers::GetRelativeInfo(path, curr_state, obj_info);
  int curr_index = params.rollOutNumber / 2 + floor(obj_info.perp_distance / params.rollOutDensity);
  if (curr_index < 0)
    curr_index = 0;
  else if (curr_index > params.rollOutNumber)
    curr_index = params.rollOutNumber;

  return curr_index;
}

void TrajectoryEvaluator::initializeCosts(const std::vector<std::vector<WayPoint> >& roll_outs,
                                          const PlanningParams& params, std::vector<TrajectoryCost>& trajectory_costs)
{
  trajectory_costs.clear();
  if (roll_outs.size() > 0)
  {
    TrajectoryCost tc;
    int center_index = params.rollOutNumber / 2;
    tc.lane_index = 0;
    for (unsigned int it = 0; it < roll_outs.size(); it++)
    {
      tc.index = it;
      tc.relative_index = it - center_index;
      tc.distance_from_center = params.rollOutDensity * tc.relative_index;
      tc.priority_cost = fabs(tc.distance_from_center);
      tc.closest_obj_distance = params.horizonDistance;
      if (roll_outs.at(it).size() > 0)
        tc.lane_change_cost = roll_outs.at(it).at(0).laneChangeCost;
      trajectory_costs.push_back(tc);
    }
  }
}

void TrajectoryEvaluator::initializeSafetyPolygon(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
                                                  const VehicleState& vehicle_state, const double& c_lateral_d,
                                                  const double& c_long_front_d, const double& c_long_back_d,
                                                  PolygonShape& car_border)
{
  PlannerHNS::Mat3 inv_rotation_mat(curr_state.pos.a - M_PI_2);
  PlannerHNS::Mat3 inv_translation_mat(curr_state.pos.x, curr_state.pos.y);

  double corner_slide_distance = c_lateral_d / 2.0;
  double ratio_to_angle = corner_slide_distance / car_info.max_steer_angle;
  double slide_distance = vehicle_state.steer * ratio_to_angle;

  GPSPoint bottom_left(-c_lateral_d, -c_long_back_d, curr_state.pos.z, 0);
  GPSPoint bottom_right(c_lateral_d, -c_long_back_d, curr_state.pos.z, 0);

  GPSPoint top_right_car(c_lateral_d, car_info.wheel_base / 3.0 + car_info.length / 3.0, curr_state.pos.z, 0);
  GPSPoint top_left_car(-c_lateral_d, car_info.wheel_base / 3.0 + car_info.length / 3.0, curr_state.pos.z, 0);

  GPSPoint top_right(c_lateral_d - slide_distance, c_long_front_d, curr_state.pos.z, 0);
  GPSPoint top_left(-c_lateral_d - slide_distance, c_long_front_d, curr_state.pos.z, 0);

  bottom_left = inv_rotation_mat * bottom_left;
  bottom_left = inv_translation_mat * bottom_left;

  top_right = inv_rotation_mat * top_right;
  top_right = inv_translation_mat * top_right;

  bottom_right = inv_rotation_mat * bottom_right;
  bottom_right = inv_translation_mat * bottom_right;

  top_left = inv_rotation_mat * top_left;
  top_left = inv_translation_mat * top_left;

  top_right_car = inv_rotation_mat * top_right_car;
  top_right_car = inv_translation_mat * top_right_car;

  top_left_car = inv_rotation_mat * top_left_car;
  top_left_car = inv_translation_mat * top_left_car;

  car_border.points.clear();
  car_border.points.push_back(bottom_left);
  car_border.points.push_back(bottom_right);
  car_border.points.push_back(top_right_car);
  car_border.points.push_back(top_right);
  car_border.points.push_back(top_left);
  car_border.points.push_back(top_left_car);
}

TrajectoryCost TrajectoryEvaluator::findBestTrajectory(const PlanningParams& params,
                                                       std::vector<TrajectoryCost>& trajectory_costs)
{
  TrajectoryCost best_trajectory;
  best_trajectory.bBlocked = true;
  best_trajectory.closest_obj_distance = params.horizonDistance;
  best_trajectory.closest_obj_velocity = 0;
  best_trajectory.index = -1;

  int smallestIndex = -1;
  double smallestCost = DBL_MAX;
  double smallestDistance = DBL_MAX;
  double velo_of_next = 0;
//      bool bAllFree = true;

std::cout << "Trajectory Costs Log " << " --------------------- " << std::endl;
  for (unsigned int ic = 0; ic < trajectory_costs.size(); ic++)
  {
    std::cout << trajectory_costs.at(ic).ToString();
    if (!trajectory_costs.at(ic).bBlocked && trajectory_costs.at(ic).cost < smallestCost)
    {
      smallestCost = trajectory_costs.at(ic).cost;
      smallestIndex = ic;
    }

    if (trajectory_costs.at(ic).closest_obj_distance < smallestDistance)
    {
      smallestDistance = trajectory_costs.at(ic).closest_obj_distance;
      velo_of_next = trajectory_costs.at(ic).closest_obj_velocity;
    }

//              if(trajectory_costs.at(ic).bBlocked)
//                      bAllFree = false;
  }

  std::cout << "Smallest (Distance, Index) " <<  smallestDistance << ", " << smallestIndex << "------------------------------------------------------------- " << std::endl;

//      if(bAllFree && smallestIndex >=0)
//              smallestIndex = params.rollOutNumber/2;

  if (smallestIndex == -1)
  {
    best_trajectory.bBlocked = true;
    best_trajectory.lane_index = 0;
    best_trajectory.index = params.rollOutNumber / 2;
    best_trajectory.closest_obj_distance = smallestDistance;
    best_trajectory.closest_obj_velocity = velo_of_next;
  }
  else if (smallestIndex >= 0)
  {
    best_trajectory = trajectory_costs.at(smallestIndex);
    best_trajectory.closest_obj_distance = smallestDistance;
  }

  return best_trajectory;
}

void TrajectoryEvaluator::calculateDistanceCosts(const PlanningParams& params, const double& c_lateral_d, const std::vector<std::vector<WayPoint> >& roll_outs, const std::vector<WayPoint>& contour_points, const std::vector<WayPoint>& trajectory_points, std::vector<TrajectoryCost>& trajectory_costs, std::vector<WayPoint>& collision_points)
{
  int center_index = params.rollOutNumber / 2;
  for(unsigned int i=0; i < roll_outs.size(); i++)
  {
    for(unsigned int j = 0; j < contour_points.size(); j++)
    {
      RelativeInfo info;
      int prev_index = 0;
      PlanningHelpers::GetRelativeInfoLimited(roll_outs.at(i), contour_points.at(j), info, prev_index);

      double actual_lateral_distance = fabs(info.perp_distance)+0.01; //add small distance so this never become zero
      double actual_longitudinal_distance = info.from_back_distance + roll_outs.at(i).at(info.iBack).cost + 0.01; //add small distance so this never become zero
      if(actual_lateral_distance < c_lateral_d && !info.bAfter && !info.bBefore) // collision point
      {
        collision_points.push_back(info.perp_point);
        double lateral_cost = 1.0/actual_lateral_distance;
        double longitudinal_cost = 1.0/actual_longitudinal_distance;

        //if(trajectory_costs.at(i).lateral_cost < lateral_cost)
          trajectory_costs.at(i).lateral_cost += lateral_cost;

        //if(trajectory_costs.at(i).longitudinal_cost < longitudinal_cost)
          trajectory_costs.at(i).longitudinal_cost  += longitudinal_cost;

//        if(trajectory_costs.at(i).closest_obj_distance > actual_longitudinal_distance)
//          trajectory_costs.at(i).closest_obj_distance = actual_longitudinal_distance;
      }
    }
  }
}










































}
