#ifndef VEHICLE_H
#define VEHICLE_H

#include <utility>
#include <vector>

class VehiclePosition {
public:
  VehiclePosition(double s, double d, double speed);

  const double &getS() const;

  const double &getD() const;

  double getYaw() const;

  double getSpeed() const;

  void setSpeed(double speed);

  void setYaw(double yaw);

private:
  using Coords = std::pair<double, double>;
  Coords m_frenet;
  double m_speed;
  double m_yaw;
};

using Trajectory = std::vector<VehiclePosition>;

#endif // VEHICLE_H
