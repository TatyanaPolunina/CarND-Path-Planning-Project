#ifndef ACCELERATIONLIMITCOST_H
#define ACCELERATIONLIMITCOST_H

#include "CostFunction.h"
#include "../roadoptions.h"

class AccelerationLimitCost : public CostFunction
{
public:
    AccelerationLimitCost(const RoadOptions& options);

    // CostFunction interface
public:
    double calculateCost(const VehiclePosition &currentState, const Trajectory &trajectory, const std::vector<double> &x, const std::vector<double> &y, const std::vector<VehiclePosition> &otherVehicles) const override;
private:
    double m_limit;
};

#endif // ACCELERATIONLIMITCOST_H
