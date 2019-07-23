#ifndef STATE_H
#define STATE_H

#include "../VehiclePosition.h"
#include <vector>

class State
{
public:
   virtual std::vector<VehiclePosition> generateTrajectory(const VehiclePosition& current_state) const = 0;
   virtual bool isStatePossible(const VehiclePosition& current_state) const = 0;
   ~State() {}
protected:
    double m_trajectory_interval;
    double m_point_interval;
};

#endif // STATE_H
