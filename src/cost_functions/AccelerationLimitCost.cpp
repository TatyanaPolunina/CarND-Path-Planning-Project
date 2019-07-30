#include "AccelerationLimitCost.h"

#include <iostream>
#include <cmath>

AccelerationLimitCost::AccelerationLimitCost(const RoadOptions &options)
    : m_limit(options.acc_limit) {}

double AccelerationLimitCost::calculateCost(
    const VehiclePosition &currentState, const Trajectory &trajectory,
    const std::vector<double> &x, const std::vector<double> &y,
    const std::vector<VehiclePosition> &otherVehicles) const
{
    std::vector<double> vx;
    std::vector<double> vy;

    for (std::size_t i = x.size() - 1; i >= 1; --i)
    {
        double x_val = x[i];
        double x_prev = x[i - 1];
        double y_val = y[i];
        double y_prev = y[i - 1];
        vx.push_back(x_val - x_prev);
        vy.push_back(y_val - y_prev);

    }

    for (std::size_t i = vx.size() - 1; i >= 1; --i)
    {
        double vx_val = vx[i];
        double vx_prev = vx[i - 1];
        double vy_val = vy[i];
        double vy_prev = vy[i - 1];
        double car_acc = sqrt((vx_val - vx_prev) * (vx_val - vx_prev) +
                     (vy_val - vy_prev) * (vy_val - vy_prev)) /
                .02;

        if (car_acc > m_limit)
        {
            std::cout <<car_acc <<std::endl;
            return 1;
        }
    }
    return 0;

}
