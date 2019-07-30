#include "ChangeSpeedState.h"
#include <iostream>
#include <sstream>

ChangeSpeedState::ChangeSpeedState(const RoadOptions &options,
                                   double acceleration)
    : State(options), m_acceleration(acceleration) {}

std::vector<VehiclePosition> ChangeSpeedState::generateTrajectory(
    const VehiclePosition &current_state) const {
  double newV =
      std::min(m_options.speed_limit,
               current_state.getSpeed() + m_acceleration * m_point_interval);
  double sPos = current_state.getS() + newV * m_point_interval;
  double lane_center =
      m_options.getLaneCenter(m_options.getLaneNumber(current_state.getD()));
  std::vector<VehiclePosition> trajectory;
  double num_points = 0;
  while (num_points++ < 50) {
    trajectory.emplace_back(sPos, lane_center, newV);
    newV = std::max(newV + m_acceleration * m_point_interval, 0.0);
    sPos = sPos + newV * m_point_interval;
  }
  return trajectory;
}

bool ChangeSpeedState::isStatePossible(
    const VehiclePosition &current_state) const {
  double new_speed =
      current_state.getSpeed() + m_acceleration * m_point_interval;
  return new_speed > 0;
}

std::string ChangeSpeedState::getName() const {
  std::stringstream str;
  str << "Change speed state " << m_acceleration;
  return str.str();
}
