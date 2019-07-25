#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "../VehiclePosition.h"
#include <vector>

class CostFunction {
public:
  virtual double calculateCost(const VehiclePosition &currentState,
                               const Trajectory& trajectory,
                               const std::vector<VehiclePosition> &otherVehicles) const = 0;
};

#endif // COSTFUNCTION_H
