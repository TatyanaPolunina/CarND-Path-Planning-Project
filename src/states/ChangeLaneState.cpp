#include "ChangeLaneState.h"

ChangeLaneState::ChangeLaneState(ChangeLaneState::LaneDirection direction, int num_lanes, double lane_width)
    : m_direction(direction)
    , m_num_lanes(num_lanes)
    , m_lane_width(lane_width)
{
}


std::vector<VehiclePosition> ChangeLaneState::generateTrajectory(const VehiclePosition &current_state) const
{
}

bool ChangeLaneState::isStatePossible(const VehiclePosition &current_state) const
{
    int direction = (m_direction == DIR_RIGHT)? 1: -1;
    int new_lane = getCurrentLane(current_state.getCoords().second) + direction;
    return new_lane >= 0 && new_lane < m_num_lanes;
}



int ChangeLaneState::getCurrentLane(double d) const
{
    return  static_cast<int>(d / m_lane_width);
}
