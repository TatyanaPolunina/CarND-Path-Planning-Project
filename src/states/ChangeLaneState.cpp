#include "ChangeLaneState.h"
#include <iostream>

ChangeLaneState::ChangeLaneState(ChangeLaneState::LaneDirection direction,
                                 const RoadOptions& options, double acceleration)
    : State(options), m_direction(direction), m_acceleration(acceleration) {}

std::vector<VehiclePosition> ChangeLaneState::generateTrajectory(
    const VehiclePosition &current_state) const {
  int direction = (m_direction == DIR_RIGHT) ? 1 : -1;
  int new_lane = m_options.getLaneNumber(current_state.getD()) + direction;

  double newV = current_state.getSpeed();
  double sPos = current_state.getS() + newV * m_point_interval;
  double lane_center = m_options.getLaneCenter(new_lane);
  VehiclePosition next_pos(sPos, lane_center, newV);
  double dist = next_pos.getS() - current_state.getS();
  std::vector<VehiclePosition> trajectory;
  while (dist < m_trajectory_dist) {
    trajectory.push_back(next_pos);
    if (dist < m_trajectory_dist / 2) {
      newV = next_pos.getSpeed();
    } else {
      newV = std::min(m_options.speed_limit,
                      next_pos.getSpeed() + m_acceleration * m_point_interval);
    }

    double sPos = next_pos.getS() + newV * m_point_interval;
    next_pos = VehiclePosition(sPos, lane_center, newV);
    dist = next_pos.getS() - current_state.getS();
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
