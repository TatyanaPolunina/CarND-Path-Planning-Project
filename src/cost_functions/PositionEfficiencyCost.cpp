#include "PositionEfficiencyCost.h"

PositionEfficiencyCost::PositionEfficiencyCost(const RoadOptions &options)
    : m_road_options(options) {}

double PositionEfficiencyCost::calculateCost(
    const VehiclePosition &currentState, const Trajectory &trajectory,
    const std::vector<VehiclePosition> &otherVehicles) const {
  double s = trajectory.back().getS();
  while (s > m_road_options.max_s) {
    s -= m_road_options.max_s;
  }
  return 1 - trajectory.back().getS() / m_road_options.max_s;
}
