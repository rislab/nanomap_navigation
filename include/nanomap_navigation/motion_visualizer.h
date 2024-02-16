#ifndef MOTION_VISUALIZER_H
#define MOTION_VISUALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <nanomap_navigation/motion_selector.h>

class MotionVisualizer {
public:
	
	MotionVisualizer() {};

	void initialize(MotionSelector* motion_selector, ros::NodeHandle & nh, size_t* best_traj_index, double const& final_time) {
		this->motion_selector = motion_selector;
		this->nh = nh;
		this->best_traj_index = best_traj_index;
		this->final_time = final_time;
		for (size_t i = 0; i < motion_selector->getNumMotions(); i++) {
      collision_probabilities.push_back(0.0);
    }

		gaussian_pub = nh.advertise<visualization_msgs::Marker>( "gaussian_visualization", 0 );
    collision_pub = nh.advertise<visualization_msgs::Marker>( "collision_visualization", 0 );

		initializeDrawingPaths();
		createSamplingTimeVector();
	};

  void UpdateTimeHorizon(double final_time);
  void createSamplingTimeVector();
  void initializeDrawingPaths();

  void drawAll();
  void drawGaussianPropagation(int id, Vector3 position, Vector3 sigma);
  void drawFinalStoppingPosition(int id, Vector3 position);
  void drawCollisionIndicator(int const& id, Vector3 const& position, double const& collision_prob);
  void setCollisionProbabilities(std::vector<double> const& collision_probabilities) {
  	this->collision_probabilities = collision_probabilities;
  }

private:

	std::string drawing_frame = "rocky0704/base";//"ortho_body";
	ros::NodeHandle nh;
	ros::Publisher gaussian_pub;
  ros::Publisher collision_pub;
	std::vector<ros::Publisher> action_paths_pubs;

  MotionSelector* motion_selector;

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector;
  size_t num_samples = 0;
  double start_time = 0;
  double final_time = 0;

  size_t* best_traj_index;

  std::vector<double> collision_probabilities;
};

#endif
