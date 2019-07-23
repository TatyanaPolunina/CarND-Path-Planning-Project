#include "ChangeLaneState.h"
#include "../spline.h"
#include <iostream>

ChangeLaneState::ChangeLaneState(ChangeLaneState::LaneDirection direction,
                                 int num_lanes, double lane_width,
                                 double speed_limit,
                                 double acceleration)
    : State(lane_width), m_direction(direction), m_num_lanes(num_lanes),
      m_speed_limit(speed_limit), m_acceleration(acceleration) {}

std::vector<VehiclePosition> ChangeLaneState::generateTrajectory(
    const VehiclePosition &current_state) const {
  tk::spline s;
  int direction = (m_direction == DIR_RIGHT) ? 1 : -1;
  int new_lane = getCurrentLane(current_state.getD()) + direction;
  std::vector<double> s_points({current_state.getS(),
                                current_state.getS() + m_trajectory_dist / 2,
                                current_state.getS() + m_trajectory_dist});
  std::vector<double> d_points({current_state.getD(),
                                current_state.getD() - m_lane_width / 2,
                                getLaneCenter(new_lane)});
  s.set_points(s_points, d_points);

  double newV = current_state.getSpeed();
  double sPos = current_state.getS() + newV * m_point_interval;

  VehiclePosition next_pos(sPos, s(sPos), newV);
  double dist = next_pos.getS() - current_state.getS();
  std::vector<VehiclePosition> trajectory;
  while (dist < m_trajectory_dist) {
    trajectory.push_back(next_pos);
    if (dist < m_trajectory_dist / 2) {
      newV = next_pos.getSpeed();
    } else {
      newV = std::min(m_speed_limit, next_pos.getSpeed() + m_acceleration * m_point_interval);
    }

    double sPos = next_pos.getS() + newV * m_point_interval;
    next_pos = VehiclePosition(sPos, s(sPos), newV);
    dist = next_pos.getS() - current_state.getS();
    std::cout << "dist " << dist << std::endl;
  }
  return trajectory;
}

bool ChangeLaneState::isStatePossible(
    const VehiclePosition &current_state) const {
  int direction = (m_direction == DIR_RIGHT) ? 1 : -1;
  int new_lane = getCurrentLane(current_state.getD()) + direction;
  return new_lane >= 0 && new_lane < m_num_lanes && current_state.getSpeed() > 0;
}



std::string ChangeLaneState::getName() const
{
    return "change lane state " + (m_direction == DIR_LEFT)? "left ": "right ";
}
