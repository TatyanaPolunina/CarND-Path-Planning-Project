#ifndef VEHICLE_H
#define VEHICLE_H

#include <utility>

using Coords = std::pair<double, double>;

class VehiclePosition {
public:
  VehiclePosition(double s, double d, double speed);

  const Coords &getCoords() const;

  double getYaw() const;

  double getSpeed() const;

  void setSpeed(double speed);

  void setYaw(double yaw);

private:
  Coords m_frenet;
  double m_speed;
  double m_yaw;
};

#endif // VEHICLE_H
