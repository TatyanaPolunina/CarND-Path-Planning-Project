#include "TrajectoryGenerator.h"
#include "states/ChangeLaneState.h"
#include "states/ChangeSpeedState.h"
#include <iostream>

TrajectoryGenerator::TrajectoryGenerator(const RoadOptions &roadOptions)
    : m_road_options(roadOptions) {
  // init all the possible states  
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
  m_states.push_back(std::move(move_right_lane_same_speed));
  m_states.push_back(std::move(move_left_lane_same_speed));
  m_states.push_back(std::move(increase_speed));
  m_states.push_back(std::move(decrease_speed));
  m_states.push_back(std::move(keep_lane));


}

std::vector<Trajectory> TrajectoryGenerator::generate_trajectories(
    const std::vector<VehiclePosition> &previousTrajectory) {
  using Trajectory = std::vector<VehiclePosition>;
  std::vector<Trajectory> possible_trajectories;
  const VehiclePosition &current_state = previousTrajectory.back();
  //generate all the possible trajectories
  for (const auto &state : m_states) {
    if (state->isStatePossible(current_state)) {
      possible_trajectories.push_back(state->generateTrajectory(current_state));
    }
  }

  return possible_trajectories;
}
