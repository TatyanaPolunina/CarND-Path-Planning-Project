#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <memory>
#include <vector>
#include "states/State.h"
#include "cost_functions/CostFunction.h"
#include "roadoptions.h"


using StatePtr = std::unique_ptr<State>;

/*
 * Generate all the possible trajectories and provide the best based on function costs
 *
*/
class TrajectoryGenerator {
public:
  TrajectoryGenerator(const RoadOptions &roadOptions);

  /*
   * Return the best of possible trajectories based on functions costs
   */
  std::vector<Trajectory>
  generate_trajectories(const std::vector<VehiclePosition> &previousTrajectory);

private:

  RoadOptions m_road_options;
  std::vector<StatePtr> m_states;
};

#endif // TRAJECTORYGENERATOR_H
