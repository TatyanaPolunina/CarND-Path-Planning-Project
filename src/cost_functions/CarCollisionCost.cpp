#include "CarCollisionCost.h"
#include <cmath>
#include <cstdlib>

CarCollisionCost::CarCollisionCost(const RoadOptions &options)
    : m_options(options) {}

double CarCollisionCost::calculateCost(
    const VehiclePosition &currentState, const Trajectory &nextState,
    const std::vector<double> &x,
    const std::vector<double> &y,
    const std::vector<VehiclePosition> &otherVehicles) const {
  double result_cost = 0;
  const auto &last_pos = nextState.back();
  for (const auto &vehicle : otherVehicles) {
    double speed = vehicle.getSpeed();
    double vehicle_s =
        vehicle.getS() + nextState.size() * m_options.point_interval * vehicle.getSpeed();
    double dist_to_vehicle = vehicle_s - last_pos.getS();

    if (m_options.getLaneNumber(last_pos.getD()) ==
        m_options.getLaneNumber(vehicle.getD())) {
      // check car behind
      if (dist_to_vehicle < m_influences_distance && dist_to_vehicle >= 0) {
        result_cost =
            std::max(result_cost, 1 - dist_to_vehicle / m_influences_distance);
      }
      // if lane changed check both sides
      if (m_options.getLaneNumber(currentState.getD()) !=
          m_options.getLaneNumber(last_pos.getD())) {
        double abs_dist = std::abs(dist_to_vehicle);
        if (abs_dist < m_collisions_distance && abs_dist > 0) {
          	result_cost =
             	 std::max(result_cost, 1 - abs_dist / m_collisions_distance);
        }
      }
    }
  }
  return result_cost;
}
