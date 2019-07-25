#ifndef CHANGELANESTATE_H
#define CHANGELANESTATE_H

#include "State.h"

class ChangeLaneState : public State
{
public:
    enum LaneDirection
    {
        DIR_LEFT,
        DIR_RIGHT
    };
    ChangeLaneState(LaneDirection direction, const RoadOptions& options, double acceleration);

    std::vector<VehiclePosition> generateTrajectory(const VehiclePosition &current_state) const override;
    bool isStatePossible(const VehiclePosition &current_state) const override;
    std::string getName() const override;
private:
    LaneDirection m_direction;
    double m_acceleration;
};

#endif // CHANGELANESTATE_H
