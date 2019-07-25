#ifndef STATE_H
#define STATE_H

#include "../VehiclePosition.h"
#include <vector>
#include <string>
#include "../roadoptions.h"

class State {
public:
  State(const RoadOptions& options);
  virtual std::vector<VehiclePosition>
  generateTrajectory(const VehiclePosition &current_state) const = 0;
  virtual bool isStatePossible(const VehiclePosition &current_state) const = 0;
  virtual std::string getName() const = 0;
  ~State() {}

protected:
  int getCurrentLane(double d) const;
  double getLaneCenter(int lane_num) const;
  double m_trajectory_dist = 10;
  double m_point_interval = 0.02;
  RoadOptions m_options;
};

#endif // STATE_H
