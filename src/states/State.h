#ifndef STATE_H
#define STATE_H

#include "../VehiclePosition.h"
#include <vector>
#include <string>

class State
{
public:
   State(double lane_width);
   virtual std::vector<VehiclePosition> generateTrajectory(const VehiclePosition& current_state) const = 0;
   virtual bool isStatePossible(const VehiclePosition& current_state) const = 0;
   virtual std::string getName() const = 0;
   ~State() {}
protected:
    int getCurrentLane(double d) const;
    double getLaneCenter(int lane_num) const;
    double m_trajectory_dist = 3;
    double m_point_interval = 0.02;
    double m_lane_width;
};


#endif // STATE_H
