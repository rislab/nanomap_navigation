#include "motion.h"
#include <math.h>

class AttitudeGenerator {
public:
  void setZsetpoint(double z_setpoint);
  void setZ(double z);
  void setZvelocity(double z_velocity);
  
  // this function is for debug use -- allows setting of gains during flight
  void setGains(Vector3 const& pid, double const& offset);
  void setOffset(double offest);

  double zPID(double forward_propagation_time);

  void UpdateRollPitch(double roll, double pitch);

  Vector3 generateDesiredAttitudeThrust(Vector3 const& desired_acceleration, double forward_propagation_time);

private:
  
  double z = 0.0;
  double z_setpoint = 1.2;
  
  double z_velocity = 0.0;
  double z_velocity_setpoint = 0.0;

  double actual_roll = 0.0;
  double actual_pitch = 0.0;

  double _dt = 1/100.0;
  double _max = 0.9;
  double _min = 0.4;
  double _Kp = 0.6;
  double _Ki = 0.6;
  double _Kd = 0.5;
  double _integral = 0.0;
  double _i_max = 0.07;
  double _offset = 0.75;

};
