#include "SpeedLimitCost.h"
#include <cmath>
#include <iostream>

SpeedLimitCost::SpeedLimitCost(double limit) : m_limit(limit) {}

double SpeedLimitCost::calculateCost(const VehiclePosition &currentState,
                                     const Trajectory& nextState,
                                     const std::vector<double> &x,
                                     const std::vector<double> &y,
                                     const std::vector<VehiclePosition> &otherVehicles) const {

    for (std::size_t i = x.size() - 1; i >= 1; --i)
    {
        double x_val = x[i];
        double x_prev = x[i - 1];
        double y_val = y[i];
        double y_prev = y[i - 1];
        double car_speed = sqrt((x_val - x_prev) * (x_val - x_prev) +
                         (y_val - y_prev) * (y_val - y_prev)) /
                    .02;
        if (car_speed > m_limit + 1 || car_speed < 0)
        {
            return 1;
        }
    }
    return 0;
}
