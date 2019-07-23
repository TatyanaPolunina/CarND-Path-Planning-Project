#include "SpeedLimitCost.h"

SpeedLimitCost::SpeedLimitCost(double limit) : m_limit(limit) {}

double SpeedLimitCost::calculateCost(const VehiclePosition &currentState,
                                     const VehiclePosition& nextState,
                                     const std::vector<VehiclePosition> &otherVehicles) const {
  return (nextState.getSpeed() > m_limit) ? 0 : 1;
}
