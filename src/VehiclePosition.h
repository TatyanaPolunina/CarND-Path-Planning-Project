#ifndef VEHICLE_H
#define VEHICLE_H

#include <utility>

using Coords = std::pair<double, double>;

class VehiclePosition {
public:
  VehiclePosition(double x, double y, double s, double d);

  VehiclePosition(double x, double y);

  const Coords &getCoords() const;

  const Coords &getFrenet() const;

  double getYaw() const;

  double getSpeed() const;

  void setSpeed(double speed);

  void setYaw(double yaw);

private:
  Coords m_cartesian;
  Coords m_frenet;
  double m_speed;
  double m_yaw;
};

#endif // VEHICLE_H
