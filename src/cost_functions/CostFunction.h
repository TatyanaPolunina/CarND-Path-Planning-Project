#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "../VehiclePosition.h"
#include <vector>


/*
 * Base cost function to define the best trajectory
 * cost should be in [0,1] interval
 * best trajectory should be chosen with minimum cost
 */
class CostFunction {
public:
  virtual double
  calculateCost(const VehiclePosition &currentState,
                const Trajectory &trajectory,
                const std::vector<VehiclePosition> &otherVehicles) const = 0;
};

#endif // COSTFUNCTION_H
