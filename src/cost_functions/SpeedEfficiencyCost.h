#ifndef SPEEDEFFICIENCYCOST_H
#define SPEEDEFFICIENCYCOST_H

#include "CostFunction.h"

/*
 * check that we go with the maximum possible speed
 */
class SpeedEfficiencyCost : public CostFunction {
public:
  SpeedEfficiencyCost(double speed_limit);

  double calculateCost(
      const VehiclePosition &currentState, const Trajectory &nextState,
      const std::vector<double> &x,
      const std::vector<double> &y,
      const std::vector<VehiclePosition> &otherVehicles) const override;

private:
  double m_speed_limit;
};

#endif // SPEEDEFFICIENCYCOST_H
