#include "State.h"



State::State(double lane_width)
    : m_lane_width(lane_width)
{

}

int State::getCurrentLane(double d) const
{
    return static_cast<int>(d / m_lane_width);
}

double State::getLaneCenter(int lane_num) const
{
    return lane_num * m_lane_width + m_lane_width / 2.0;
}
