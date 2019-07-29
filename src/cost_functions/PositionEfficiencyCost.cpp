#include "PositionEfficiencyCost.h"

PositionEfficiencyCost::PositionEfficiencyCost(const RoadOptions &options)
    : m_road_options(options) {}

double PositionEfficiencyCost::calculateCost(
    const VehiclePosition &currentState, const Trajectory &trajectory,
    const std::vector<VehiclePosition> &otherVehicles) const {
  return 1 - trajectory.back().getS() / m_road_options.max_s;
}
