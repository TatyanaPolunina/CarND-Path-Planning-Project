#include "BestCostStrategy.h"
#include "SpeedEfficiencyCost.h"
#include "SpeedLimitCost.h"
#include "CarCollisionCost.h"
#include "PositionEfficiencyCost.h"
#include "AccelerationLimitCost.h"

static const double SAFETY_WEIGHT = 100;
static const double EFFITIENCY_WEIGHT = 1;


BestCostStrategy::BestCostStrategy(const RoadOptions& options)
    : m_road_options(options)
{
    // define all the presented cost function with corresponded weights
    std::unique_ptr<SpeedEfficiencyCost> speed_efficiency(
        new SpeedEfficiencyCost(m_road_options.speed_limit));
    std::unique_ptr<SpeedLimitCost> speed_limit(
        new SpeedLimitCost(m_road_options.speed_limit));
    std::unique_ptr<CarCollisionCost> collision(
        new CarCollisionCost(m_road_options));
    std::unique_ptr<PositionEfficiencyCost> pos_efficiency(
        new PositionEfficiencyCost(m_road_options));
    std::unique_ptr<AccelerationLimitCost> acc_limit(
        new AccelerationLimitCost(m_road_options));
    m_functions.push_back(std::make_pair(SAFETY_WEIGHT, std::move(collision)));
    m_functions.push_back(std::make_pair(SAFETY_WEIGHT, std::move(speed_limit)));
    m_functions.push_back(std::make_pair(SAFETY_WEIGHT, std::move(acc_limit)));
    m_functions.push_back(
        std::make_pair(EFFITIENCY_WEIGHT, std::move(speed_efficiency)));
    m_functions.push_back(
                std::make_pair(EFFITIENCY_WEIGHT, std::move(pos_efficiency)));
}

double BestCostStrategy::calculateCost(const VehiclePosition& current_state,
                                       const Trajectory &trajectory, const std::vector<double> &x, const std::vector<double> &y,
                                       const std::vector<VehiclePosition> &other_vehicles)
{
    double cost = 0;
    for (const auto &weigted_func : m_functions) {
      cost += weigted_func.first *
              weigted_func.second->calculateCost(current_state, trajectory,x,y,
                                                 other_vehicles);
    }
    return cost;
}
