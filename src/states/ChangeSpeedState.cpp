#include "ChangeSpeedState.h"
#include <iostream>

ChangeSpeedState::ChangeSpeedState(const RoadOptions& options, double acceleration)
    : State (options)
    , m_acceleration(acceleration)
{
}


std::vector<VehiclePosition> ChangeSpeedState::generateTrajectory(const VehiclePosition &current_state) const
{
    double newV = std::min(m_speed_limit, current_state.getSpeed() + m_acceleration * m_point_interval);
    double sPos = current_state.getS() + newV * m_point_interval;
    double dist = sPos - current_state.getS();
    double lane_center = m_options.getLaneCenter(m_options.getLaneNumber(current_state.getD()));
    double dPos = lane_center;

    VehiclePosition next_pos(sPos, dPos, newV);

    std::vector<VehiclePosition> trajectory;
    while (dist < m_trajectory_dist)
    {
        trajectory.push_back(next_pos);
        newV = std::min(m_speed_limit, next_pos.getSpeed() + m_acceleration * m_point_interval);
        double sPos = next_pos.getS() + newV * m_point_interval;
        dPos = lane_center;
        next_pos = VehiclePosition(sPos, dPos, newV);
        dist = next_pos.getS() - current_state.getS();
    }
    return trajectory;
}


bool ChangeSpeedState::isStatePossible(const VehiclePosition &current_state) const
{
  double new_speed = current_state.getSpeed() + m_acceleration * m_point_interval;
  return new_speed <= m_options.speed_limit && new_speed >= 0;
}


std::string ChangeSpeedState::getName() const
{
    return "Change speed state";
}
