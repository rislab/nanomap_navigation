#include <ros/ros.h>

#include <nanomap_navigation/motion_selector.h>
#include <nanomap_navigation/attitude_generator.h>
#include <nanomap_navigation/motion_visualizer.h>

#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;

class NanoMapNavigationNode {
public:
  NanoMapNavigationNode() : nh("~") {
    
    // Get from param server for initialization
		double acceleration_interpolation_min;
		double soft_top_speed;
    double speed_at_acceleration_max;
    double acceleration_interpolation_max;

    pu::get("acceleration_interpolation_min", acceleration_interpolation_min);
    pu::get("soft_top_speed", soft_top_speed);
    pu::get("speed_at_acceleration_max", speed_at_acceleration_max);
    pu::get("acceleration_interpolation_max", acceleration_interpolation_max);

		motion_selector.InitializeLibrary(use_3d_library, final_time, soft_top_speed, acceleration_interpolation_min, speed_at_acceleration_max, acceleration_interpolation_max);

		motion_selector.SetNominalFlightAltitude(flight_altitude);
		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();

		motion_visualizer.initialize(&motion_selector, nh, &best_traj_index, final_time);
		nanomap_visualizer.Initialize(nh);
  }
private:
 
  // To initialize library
	bool use_3d_library = false;
	double final_time = 1.5;


	MotionSelector motion_selector;
	AttitudeGenerator attitude_generator;
	NanoMapVisualizer nanomap_visualizer;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nanomap_navigation");

  // TODO
  nav_node = NanoMapNavigationNode();

  // Pseudocode
  /*

  */

  ros::spin();
  return 0;
}