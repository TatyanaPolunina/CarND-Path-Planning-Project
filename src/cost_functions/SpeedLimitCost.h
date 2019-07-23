#ifndef SPEEDLIMITCOST_H
#define SPEEDLIMITCOST_H

#include "CostFunction.h"

class SpeedLimitCost : public CostFunction
{
public:
    SpeedLimitCost(double maxLimit);

    double calculateCost(const VehiclePosition &currentState,
                         const VehiclePosition &nextState,
                         const std::vector<VehiclePosition> &otherVehicles) override;
private:
    double m_limit;
};

#endif // SPEEDLIMITCOST_H
