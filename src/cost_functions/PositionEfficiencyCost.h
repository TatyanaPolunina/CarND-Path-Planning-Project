#ifndef POSITIONEFFICIENCYCOST_H
#define POSITIONEFFICIENCYCOST_H

#include "CostFunction.h"
#include "../roadoptions.h"

class PositionEfficiencyCost : public CostFunction
{
public:
    PositionEfficiencyCost(const RoadOptions& options);
    double calculateCost(const VehiclePosition &currentState, const Trajectory &trajectory, const std::vector<VehiclePosition> &otherVehicles) const override;
private:
    RoadOptions m_road_options;
};

#endif // POSITIONEFFICIENCYCOST_H
