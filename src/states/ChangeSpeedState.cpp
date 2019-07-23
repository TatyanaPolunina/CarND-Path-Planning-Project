#include "ChangeSpeedState.h"
#include "../spline.h"
#include <iostream>

ChangeSpeedState::ChangeSpeedState(double lane_width, double acceleration, double speed_limit)
    : State (lane_width)
    , m_acceleration(acceleration)
    , m_speed_limit(speed_limit)
{
}


std::vector<VehiclePosition> ChangeSpeedState::generateTrajectory(const VehiclePosition &current_state) const
{
    tk::spline s;
    std::vector<double> s_points({current_state.getS(), current_state.getS() + m_trajectory_dist});
    std::vector<double> d_points({current_state.getD(), getLaneCenter(getCurrentLane(current_state.getD()))});
    s.set_points(s_points, d_points);

    double newV = current_state.getSpeed() + m_acceleration * m_point_interval;
    double sPos = current_state.getS() + newV * m_point_interval;

    VehiclePosition next_pos(sPos, s(sPos), newV);
    double dist = next_pos.getS() - current_state.getS();
    std::vector<VehiclePosition> trajectory;
    while (dist < m_trajectory_dist)
    {
        trajectory.push_back(next_pos);
        std::cout << next_pos.getS() << ' ' << next_pos.getD() << std::endl;
        newV = std::min(m_speed_limit, next_pos.getSpeed() + m_acceleration * m_point_interval);
        double sPos = next_pos.getS() + newV * m_point_interval;
        next_pos = VehiclePosition(sPos, s(sPos), newV);
        dist = next_pos.getS() - current_state.getS();
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
