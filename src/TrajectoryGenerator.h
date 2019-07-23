#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <memory>
#include <vector>
#include "states/State.h"
#include "cost_functions/CostFunction.h"

struct RoadOptions
{
    double lane_width;
    double lane_number;
    double speed_limit;
};

using StatePtr = std::unique_ptr<State>;
using CostFunctionPtr = std::unique_ptr<CostFunction>;


class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const RoadOptions& roadOptions);
    std::vector<VehiclePosition> generate_trajectory(const std::vector<VehiclePosition>& previousTrajectory,
                                                     const std::vector<VehiclePosition>& other_vehicles);
private:

    double calculateCost(    const VehiclePosition &current_state,
                             const std::vector<VehiclePosition> &trajectory,
                             const std::vector<VehiclePosition> &other_vehicles);

    RoadOptions m_road_options;
    std::vector<StatePtr> m_states;
    using WeightedCost = std::pair<double, CostFunctionPtr>;
    std::vector<WeightedCost> m_functions;

};

#endif // TRAJECTORYGENERATOR_H
