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
    ChangeLaneState(LaneDirection direction, int num_lanes, double lane_width);

    std::vector<VehiclePosition> generateTrajectory(const VehiclePosition &current_state) const override;
    bool isStatePossible(const VehiclePosition &current_state) const override;
private:
    int getCurrentLane(double d) const;
    LaneDirection m_direction;
    int m_num_lanes;
    double m_lane_width;

};

#endif // CHANGELANESTATE_H
