#include "VehiclePosition.h"

VehiclePosition::VehiclePosition(double x, double y, double s, double d)
    : m_cartesian(x,y)
    , m_frenet(s,d)
    , m_speed(0)
    , m_yaw(0)
{

}

const Coords& VehiclePosition::getCoords() const
{
    return  m_cartesian;
}

const Coords& VehiclePosition::getFrenet() const
{
    return m_frenet;
}

double VehiclePosition::getYaw() const
{
    return m_yaw;
}

double VehiclePosition::getSpeed() const
{
    return m_speed;
}

void VehiclePosition::setSpeed(double speed)
{
    m_speed = speed;
}

void VehiclePosition::setYaw(double yaw)
{
    m_yaw = yaw;
}
