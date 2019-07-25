#include "SpeedLimitCost.h"

SpeedLimitCost::SpeedLimitCost(double limit) : m_limit(limit) {}

double SpeedLimitCost::calculateCost(const VehiclePosition &currentState,
                                     const Trajectory& nextState,
                                     const std::vector<VehiclePosition> &otherVehicles) const {
  return (nextState.back().getSpeed() > m_limit) ? 0 : 1;
}
