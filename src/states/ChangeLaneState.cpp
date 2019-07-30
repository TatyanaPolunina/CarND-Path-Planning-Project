#include "ChangeLaneState.h"
#include <iostream>
#include <cmath>

ChangeLaneState::ChangeLaneState(ChangeLaneState::LaneDirection direction,
                                 const RoadOptions &options,
                                 double acceleration)
    : State(options), m_direction(direction), m_acceleration(acceleration) {}

std::vector<VehiclePosition> ChangeLaneState::generateTrajectory(
    const VehiclePosition &current_state) const {
  int num_points = 50;
  int direction = (m_direction == DIR_RIGHT) ? 1 : -1;
  int new_lane = m_options.getLaneNumber(current_state.getD()) + direction;
  double newV = std::min(current_state.getSpeed() + m_acceleration * m_options.point_interval, m_options.speed_limit);
  double dist_s = newV * m_options.point_interval;
  double cos_alpha = m_options.lane_width / num_points / dist_s;
  double sin_alpha = sqrt(1 - cos_alpha * cos_alpha);
  double diff_s = dist_s * sin_alpha;

  double lane_center = m_options.getLaneCenter(new_lane);
  std::vector<VehiclePosition> trajectory;
  double next_s = current_state.getS() + diff_s;
  for (int i = 0; i < num_points; ++i) {
    trajectory.emplace_back(next_s, lane_center, newV);
    dist_s = newV * m_options.point_interval;
    diff_s = dist_s * sin_alpha;
    next_s += diff_s;
  }
  return trajectory;
}

bool ChangeLaneState::isStatePossible(
    const VehiclePosition &current_state) const {
  int direction = (m_direction == DIR_RIGHT) ? 1 : -1;
  int new_lane = m_options.getLaneNumber(current_state.getD()) + direction;
  return new_lane >= 0 && new_lane < m_options.lane_number &&
         current_state.getSpeed() > 0;
}

std::string ChangeLaneState::getName() const {
  return "change lane state " + (m_direction == DIR_LEFT) ? "left " : "right ";
}
