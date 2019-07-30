#ifndef STATE_H
#define STATE_H

#include "../VehiclePosition.h"
#include <vector>
#include <string>
#include "../roadoptions.h"


/*
 * The base class for defining possible state (trajectory)
 */
class State {
public:
  State(const RoadOptions &options);

  /*
   * state trajectory generation
   */
  virtual std::vector<VehiclePosition>
  generateTrajectory(const VehiclePosition &current_state) const = 0;

  /*
   * Check if current state is possible in this current position
   */
  virtual bool isStatePossible(const VehiclePosition &current_state) const = 0;

  /*
   * state name for the debug purposes
   */
  virtual std::string getName() const = 0;
  ~State() {}

protected:
  int getCurrentLane(double d) const;
  double getLaneCenter(int lane_num) const;
  RoadOptions m_options;
};

#endif // STATE_H
