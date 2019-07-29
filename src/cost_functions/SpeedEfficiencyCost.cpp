#include "SpeedEfficiencyCost.h"

SpeedEfficiencyCost::SpeedEfficiencyCost(double speed_limit)
    : m_speed_limit(speed_limit) {}

double SpeedEfficiencyCost::calculateCost(
    const VehiclePosition &currentState, const Trajectory &nextState,
    const std::vector<VehiclePosition> &otherVehicles) const {
  return (m_speed_limit - nextState.back().getSpeed()) / m_speed_limit;
}
