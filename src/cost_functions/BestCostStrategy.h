#ifndef BESTCOSTSTRATEGY_H
#define BESTCOSTSTRATEGY_H

#include "../roadoptions.h"
#include <utility>
#include <memory>
#include <vector>
#include "../VehiclePosition.h"
#include "CostFunction.h"

using CostFunctionPtr = std::unique_ptr<CostFunction>;

class BestCostStrategy
{
public:
    BestCostStrategy(const RoadOptions& options);
    double calculateCost(const VehiclePosition& current_state,
                         const Trajectory &trajectory,
                         const std::vector<double> &x,
                         const std::vector<double> &y,
                         const std::vector<VehiclePosition> &other_vehicles);
private:
    RoadOptions m_road_options;
    using WeightedCost = std::pair<double, CostFunctionPtr>;
    std::vector<WeightedCost> m_functions;
};

#endif // BESTCOSTSTRATEGY_H
