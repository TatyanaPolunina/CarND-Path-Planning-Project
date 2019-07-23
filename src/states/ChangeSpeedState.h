#ifndef CHANGESPEEDSTATE_H
#define CHANGESPEEDSTATE_H

#include "State.h"

class ChangeSpeedState : public State
{
public:
    ChangeSpeedState(double lane_width, double acceleration, double speed_limit);
    std::vector<VehiclePosition> generateTrajectory(const VehiclePosition &current_state) const override;
    bool isStatePossible(const VehiclePosition &current_state) const override;
    std::string getName() const override;
private:
    double m_acceleration;
    double m_speed_limit;
};

#endif // CHANGESPEEDSTATE_H
