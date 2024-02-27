#ifndef MOTION_LIBRARY_H
#define MOTION_LIBRARY_H

#include <nanomap_navigation/motion.h>
#include <vector>

#include <string>
#include <map>
#include <random>
#include <math.h>
#include <cmath>

class MotionLibrary {
public:

  void InitializeLibrary(bool use_3d_library, double a_max_horizontal, double, double);
  void BuildMotionsSamplingAroundHorizontalCircle(double vertical_acceleration, double horizontal_acceleration_radius, size_t num_samples_around_circle);

  void setInitialVelocity(Vector3 const& initialVelocity);

  void setRollPitch(double const& roll, double const& pitch) {
    this->roll = roll;
    this->pitch = pitch;
    updateInitialAcceleration();
  };
  void setRollPitchDirectly(double const& roll, double const& pitch) {
    this->roll = roll;
    this->pitch = pitch;
  };
  void setThrust(double const& thrust) {
    this->thrust = thrust;
  };

  void updateInitialAcceleration();
  Vector3 getInitialAcceleration() const{
    return initial_acceleration;
  }

  void setInitialAcceleration(const Vector3& initial_accel);
  void setMaxAccelerationTotal(double max_accel);
  void setMinSpeedAtMaxAccelerationTotal(double speed);

  void setBestAccelerationMotion(Vector3 best_acceleration);

  void UpdateMaxAcceleration(double speed);
  double ComputeNewMaxAcceleration(double speed);

  Motion getMotionFromIndex(size_t index);
  size_t getNumMotions();
  Vector3 getSigmaAtTime(double const& t);
  Vector3 getInverseSigmaAtTime(double const& t);

  std::vector<Motion>::const_iterator GetMotionIteratorBegin() const {
	return motions.begin(); 
  };

  std::vector<Motion>::const_iterator GetMotionIteratorEnd() const {
	return motions.end(); 
  };

  std::vector<Motion>::iterator GetMotionNonConstIteratorBegin() {
  return motions.begin(); 
  };

  std::vector<Motion>::iterator GetMotionNonConstIteratorEnd() {
  return motions.end(); 
  };

  double getNewMaxAcceleration() const;



private:
  
  std::vector<Motion> motions;

  Vector3 initial_velocity = Vector3(0,0,0);
  Vector3 initial_acceleration = Vector3(0,0,0);

  Vector3 initial_velocity_laser_frame = Vector3(0,0,0);
  Vector3 initial_acceleration_laser_frame = Vector3(0,0,0);

  Vector3 initial_velocity_rdf_frame = Vector3(0,0,0);
  Vector3 initial_acceleration_rdf_frame = Vector3(0,0,0);

  double roll = 0;
  double pitch = 0;
  double thrust = 0;

  std::vector<Vector3> sampled_velocities;

  double initial_max_acceleration = 0.0;
  double new_max_acceleration;

  double speed_at_acceleration_max = 5.0;
  double max_acceleration_total = 4.0;

};

#endif
