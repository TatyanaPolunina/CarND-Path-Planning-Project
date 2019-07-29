#include "VehiclePosition.h"

VehiclePosition::VehiclePosition(double s, double d, double speed)
    : m_frenet(s, d), m_speed(speed) {}

const double &VehiclePosition::getS() const { return m_frenet.first; }

const double &VehiclePosition::getD() const { return m_frenet.second; }

double VehiclePosition::getSpeed() const { return m_speed; }

void VehiclePosition::setSpeed(double speed) { m_speed = speed; }
