#include "SpeedEfficiencyCost.h"

SpeedEfficiencyCost::SpeedEfficiencyCost(int speed_limit)
    : m_speed_limit(speed_limit)
{

}

double SpeedEfficiencyCost::calculateCost(const VehiclePosition &currentState, const VehiclePosition &nextState, const std::vector<VehiclePosition> &otherVehicles) const
{
    return  1 - (m_speed_limit - nextState.getSpeed()) / m_speed_limit;
}
