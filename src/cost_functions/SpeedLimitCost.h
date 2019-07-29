#ifndef SPEEDLIMITCOST_H
#define SPEEDLIMITCOST_H

#include "CostFunction.h"

/*
 * binary cost to check speed limit
 */
class SpeedLimitCost : public CostFunction
{
public:
    SpeedLimitCost(double maxLimit);

    double calculateCost(const VehiclePosition &currentState,
                         const Trajectory &nextState,
                         const std::vector<VehiclePosition> &otherVehicles) const override;
private:
    double m_limit;
};

#endif // SPEEDLIMITCOST_H
