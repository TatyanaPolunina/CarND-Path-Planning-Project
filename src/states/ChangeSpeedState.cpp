#include "ChangeSpeedState.h"
#include <iostream>

ChangeSpeedState::ChangeSpeedState(double lane_width, double acceleration, double speed_limit)
    : State (lane_width)
    , m_acceleration(acceleration)
    , m_speed_limit(speed_limit)
{
}


std::vector<VehiclePosition> ChangeSpeedState::generateTrajectory(const VehiclePosition &current_state) const
{

    double newV = current_state.getSpeed() + m_acceleration * m_point_interval;
    double sPos = current_state.getS() + newV * m_point_interval;
    double dist = sPos - current_state.getS();
    double lane_center = getLaneCenter(getCurrentLane(sPos));
    double dPos = current_state.getD() + dist/m_trajectory_dist * (lane_center - current_state.getD());

    VehiclePosition next_pos(sPos, dPos, newV);

    std::vector<VehiclePosition> trajectory;
    while (dist < m_trajectory_dist)
    {
        trajectory.push_back(next_pos);
        std::cout << next_pos.getS() << ' ' << next_pos.getD() << std::endl;
        newV = std::min(m_speed_limit, next_pos.getSpeed() + m_acceleration * m_point_interval);
        double sPos = next_pos.getS() + newV * m_point_interval;
        dist = next_pos.getS() - current_state.getS();
        dPos = current_state.getD() + dist/m_trajectory_dist * (lane_center - current_state.getD());
        next_pos = VehiclePosition(sPos, dPos, newV);

    }
    return trajectory;
}


bool ChangeSpeedState::isStatePossible(const VehiclePosition &current_state) const
{

  double new_speed = current_state.getSpeed() + m_acceleration * m_point_interval;
  std::cout << "change speed possible" << new_speed << std::endl;
  return new_speed <= m_speed_limit && new_speed >= 0 && new_speed > 0;
}


std::string ChangeSpeedState::getName() const
{
    return "Change speed state";
}
