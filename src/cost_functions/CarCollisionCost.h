#ifndef CARCOLLISIONCOST_H
#define CARCOLLISIONCOST_H

#include "CostFunction.h"
#include "../roadoptions.h"

class CarCollisionCost : public CostFunction {
public:
  CarCollisionCost(const RoadOptions &m_options);

  double calculateCost(
      const VehiclePosition &currentState, const Trajectory &nextState,
      const std::vector<VehiclePosition> &otherVehicles) const override;

private:
  RoadOptions m_options;
  double m_collision_distance = 30;  // meters
  double m_influences_distance = 30; // meters
};

#endif // CARCOLLISIONCOST_H
