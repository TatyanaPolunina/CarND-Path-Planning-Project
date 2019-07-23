#ifndef SPEEDEFFICIENCYCOST_H
#define SPEEDEFFICIENCYCOST_H

#include "CostFunction.h"

class SpeedEfficiencyCost : public CostFunction {
public:
  SpeedEfficiencyCost(int speed_limit);

  double calculateCost(
      const VehiclePosition &currentState, const VehiclePosition &nextState,
      const std::vector<VehiclePosition> &otherVehicles) const override;

private:
  int m_speed_limit;

};

#endif // SPEEDEFFICIENCYCOST_H
