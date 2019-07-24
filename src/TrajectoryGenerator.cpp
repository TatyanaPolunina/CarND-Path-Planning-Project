#include "TrajectoryGenerator.h"
#include "states/ChangeLaneState.h"
#include "states/ChangeSpeedState.h"
#include "cost_functions/SpeedEfficiencyCost.h"
#include "cost_functions/SpeedLimitCost.h"
#include <iostream>

static const double SAFETY_WEIGHT = 1000;
static const double EFFITIENCY_COST = 100;

TrajectoryGenerator::TrajectoryGenerator(const RoadOptions &roadOptions)
    : m_road_options(roadOptions) {
  // init all the possible states
  std::unique_ptr<ChangeLaneState> move_right_lane(new ChangeLaneState(
      ChangeLaneState::DIR_RIGHT, m_road_options.lane_number,
      m_road_options.lane_width, m_road_options.speed_limit, 5));
  std::unique_ptr<ChangeLaneState> move_left_lane(
      new ChangeLaneState(ChangeLaneState::DIR_LEFT, m_road_options.lane_number,
                          m_road_options.lane_width, m_road_options.speed_limit, 5));
  std::unique_ptr<ChangeSpeedState> increase_speed(
      new ChangeSpeedState( m_road_options.lane_width, 5, m_road_options.speed_limit));
  std::unique_ptr<ChangeSpeedState> decrease_speed(
      new ChangeSpeedState( m_road_options.lane_width, -5, m_road_options.speed_limit));
  std::unique_ptr<ChangeSpeedState> keep_lane(
      new ChangeSpeedState( m_road_options.lane_width, 0, m_road_options.speed_limit));
  //m_states.push_back(std::move(move_right_lane));
  //m_states.push_back(std::move(move_left_lane));
  m_states.push_back(std::move(increase_speed));
  //m_states.push_back(std::move(decrease_speed));
  m_states.push_back(std::move(keep_lane));

  std::unique_ptr<SpeedEfficiencyCost> speed_efficiency(
      new SpeedEfficiencyCost(m_road_options.speed_limit));
  std::unique_ptr<SpeedLimitCost> speed_limit(
      new SpeedLimitCost(m_road_options.speed_limit));
  m_functions.push_back(std::make_pair(SAFETY_WEIGHT, std::move(speed_limit)));
  m_functions.push_back(
      std::make_pair(EFFITIENCY_COST, std::move(speed_efficiency)));
}

std::vector<VehiclePosition> TrajectoryGenerator::generate_trajectory(
    const std::vector<VehiclePosition> &previousTrajectory,
    const std::vector<VehiclePosition> &other_vehicles) {
  using Trajectory = std::vector<VehiclePosition>;
  std::vector<Trajectory> possible_trajectories;
  const VehiclePosition &current_state = previousTrajectory.back();
  for (const auto &state : m_states) {
    if (state->isStatePossible(current_state)) {
      std::cout << "build trajectory for state " << state->getName() << std::endl;
      possible_trajectories.push_back(state->generateTrajectory(current_state));
    }
  }

  auto best_trajectory = possible_trajectories.begin();
  double best_cost = calculateCost(current_state, *best_trajectory, other_vehicles);
  for (auto iter = possible_trajectories.begin() + 1; iter != possible_trajectories.end(); ++iter)
  {
      double cost = calculateCost(current_state, *iter, other_vehicles);
      if (cost > best_cost)
      {
          best_cost = cost;
          best_trajectory = iter;
      }
  }
  return  *best_trajectory;
}

double TrajectoryGenerator::calculateCost(
    const VehiclePosition &current_state,
    const std::vector<VehiclePosition> &trajectory,
    const std::vector<VehiclePosition> &other_vehicles) {
  double cost = 0;
  for (const auto &weigted_func : m_functions) {
    cost += weigted_func.first *
            weigted_func.second->calculateCost(current_state, trajectory.back(),
                                               other_vehicles);
  }
  return cost;
}
