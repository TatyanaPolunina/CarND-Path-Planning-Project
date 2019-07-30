#ifndef POSITIONEFFICIENCYCOST_H
#define POSITIONEFFICIENCYCOST_H

#include "CostFunction.h"
#include "../roadoptions.h"

/*
 * the better trajectory should have the bigger S coordinate
 */
class PositionEfficiencyCost : public CostFunction {
public:
  PositionEfficiencyCost(const RoadOptions &options);
  double calculateCost(
      const VehiclePosition &currentState, const Trajectory &trajectory,
      const std::vector<double> &x,
      const std::vector<double> &y,
      const std::vector<VehiclePosition> &otherVehicles) const override;

private:
  RoadOptions m_road_options;
};

#endif // POSITIONEFFICIENCYCOST_H
