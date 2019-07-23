#include "ChangeSpeedState.h"

ChangeSpeedState::ChangeSpeedState(double acceleration, double speed_limit)
    : m_acceleration(acceleration)
    , m_speed_limit(speed_limit)
{
}


std::vector<VehiclePosition> ChangeSpeedState::generateTrajectory(const VehiclePosition &current_state) const
{
}

bool ChangeSpeedState::isStatePossible(const VehiclePosition &current_state) const
{

  double new_speed = current_state.getSpeed() + m_acceleration * m_trajectory_interval;
  return new_speed <= m_speed_limit && new_speed >= 0;
}
