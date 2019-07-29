#ifndef CHANGESPEEDSTATE_H
#define CHANGESPEEDSTATE_H

#include "State.h"

class ChangeSpeedState : public State {
public:
  ChangeSpeedState(const RoadOptions &options, double acceleration);
  std::vector<VehiclePosition>
  generateTrajectory(const VehiclePosition &current_state) const override;
  bool isStatePossible(const VehiclePosition &current_state) const override;
  std::string getName() const override;

private:
  double m_acceleration;
};

#endif // CHANGESPEEDSTATE_H
