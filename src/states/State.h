#ifndef STATE_H
#define STATE_H


class State
{
public:
   virtual VehicleState generateNextState(const VehicleState& current_state) = 0;
   virtual bool isStatePossible(const VehicleState& current_state) = 0;
   ~State() {}
};

#endif // STATE_H
