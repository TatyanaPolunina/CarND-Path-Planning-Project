#include "TrajectoryGenerator.h"
#include "states/ChangeLaneState.h"
#include "states/ChangeSpeedState.h"
#include "cost_functions/SpeedEfficiencyCost.h"
#include "cost_functions/SpeedLimitCost.h"
#include "cost_functions/CarCollisionCost.h"
#include "cost_functions/PositionEfficiencyCost.h"
#include <iostream>

static const double SAFETY_WEIGHT = 1000;
static const double EFFITIENCY_COST = 100;

TrajectoryGenerator::TrajectoryGenerator(const RoadOptions &roadOptions)
    : m_road_options(roadOptions) {
  // init all the possible states
  std::unique_ptr<ChangeLaneState> move_right_lane_with_acc(
      new ChangeLaneState(ChangeLaneState::DIR_RIGHT, m_road_options, 4));
  std::unique_ptr<ChangeLaneState> move_left_lane_with_acc(
      new ChangeLaneState(ChangeLaneState::DIR_LEFT, m_road_options, 4));
  std::unique_ptr<ChangeLaneState> move_right_lane_same_speed(
      new ChangeLaneState(ChangeLaneState::DIR_RIGHT, m_road_options, 0));
  std::unique_ptr<ChangeLaneState> move_left_lane_same_speed(
      new ChangeLaneState(ChangeLaneState::DIR_LEFT, m_road_options, 0));
  std::unique_ptr<ChangeSpeedState> increase_speed(
      new ChangeSpeedState(m_road_options, 4));
  std::unique_ptr<ChangeSpeedState> decrease_speed(
      new ChangeSpeedState(m_road_options, -4));
  std::unique_ptr<ChangeSpeedState> keep_lane(
      new ChangeSpeedState(m_road_options, 0));
  m_states.push_back(std::move(move_right_lane_with_acc));
  m_states.push_back(std::move(move_left_lane_with_acc));
  m_states.push_back(std::move(move_right_lane_same_speed));
  m_states.push_back(std::move(move_left_lane_same_speed));
  m_states.push_back(std::move(increase_speed));
  m_states.push_back(std::move(decrease_speed));
  m_states.push_back(std::move(keep_lane));

  // define all the presented cost function with corresponded weights
  std::unique_ptr<SpeedEfficiencyCost> speed_efficiency(
      new SpeedEfficiencyCost(m_road_options.speed_limit));
  std::unique_ptr<SpeedLimitCost> speed_limit(
      new SpeedLimitCost(m_road_options.speed_limit));
  std::unique_ptr<CarCollisionCost> collision(
      new CarCollisionCost(m_road_options));
  std::unique_ptr<PositionEfficiencyCost> pos_efficiency(
      new PositionEfficiencyCost(m_road_options));
  m_functions.push_back(std::make_pair(SAFETY_WEIGHT, std::move(collision)));
  m_functions.push_back(std::make_pair(SAFETY_WEIGHT, std::move(speed_limit)));
  m_functions.push_back(
      std::make_pair(EFFITIENCY_COST, std::move(speed_efficiency)));
  m_functions.push_back(
      std::make_pair(EFFITIENCY_COST, std::move(pos_efficiency)));
}

std::vector<VehiclePosition> TrajectoryGenerator::generate_trajectory(
    const std::vector<VehiclePosition> &previousTrajectory,
    const std::vector<VehiclePosition> &other_vehicles) {
  using Trajectory = std::vector<VehiclePosition>;
  std::vector<Trajectory> possible_trajectories;
  const VehiclePosition &current_state = previousTrajectory.back();
  //generate all the possible trajectories
  for (const auto &state : m_states) {
    if (state->isStatePossible(current_state)) {
      possible_trajectories.push_back(state->generateTrajectory(current_state));
    }
  }

  // find the best trajectory based on cost function costs
  auto best_trajectory = possible_trajectories.begin();
  double best_cost =
      calculateCost(current_state, *best_trajectory, other_vehicles);
  for (auto iter = possible_trajectories.begin() + 1;
       iter != possible_trajectories.end(); ++iter) {
    double cost = calculateCost(current_state, *iter, other_vehicles);
    if (cost < best_cost) {
      best_cost = cost;
      best_trajectory = iter;
    }
  }

  return *best_trajectory;
}

double TrajectoryGenerator::calculateCost(
    const VehiclePosition &current_state,
    const std::vector<VehiclePosition> &trajectory,
    const std::vector<VehiclePosition> &other_vehicles) {
  double cost = 0;
  for (const auto &weigted_func : m_functions) {
    cost += weigted_func.first *
            weigted_func.second->calculateCost(current_state, trajectory,
                                               other_vehicles);
  }
  return cost;
}
