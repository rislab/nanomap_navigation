#include <nanomap_navigation/motion_visualizer.h>

void MotionVisualizer::initializeDrawingPaths() {
	paths_pub = nh.advertise<visualization_msgs::Marker>("all_primitives", 1);
	best_path_pub = nh.advertise<visualization_msgs::Marker>("best_primitive", 1);
}

void MotionVisualizer::UpdateTimeHorizon(double final_time) {
	this->final_time = final_time;
	createSamplingTimeVector();
}

void MotionVisualizer::createSamplingTimeVector() {
	num_samples = 10;
	sampling_time_vector.resize(num_samples, 1);

	double sampling_time = 0;
	double sampling_interval = (final_time - start_time) / num_samples;
	for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
		sampling_time = start_time + sampling_interval*(sample_index+1);
		sampling_time_vector(sample_index) = sampling_time;
	}
}

void MotionVisualizer::drawGaussianPropagation(int id, Vector3 position, Vector3 sigma) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = drawing_frame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "gaussian_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = sigma(0)*2;
	marker.scale.y = sigma(1)*2;
	marker.scale.z = sigma(2)*2;
	marker.color.a = 0.30; // Don't forget to set the alpha!
	marker.color.r = 0.9;
	marker.color.g = 0.1;
	marker.color.b = 0.9;
	gaussian_pub.publish( marker );
}

void MotionVisualizer::drawCollisionIndicator(int const& id, Vector3 const& position, double const& collision_prob) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = drawing_frame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "collision_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = 0.8;
	marker.scale.y = 0.8;
	marker.scale.z = 0.8;
	marker.color.a = 0.15; // Don't forget to set the alpha!
	marker.color.r = collision_prob;
	marker.color.g = 1.0 - collision_prob;
	marker.color.b = 0.0;
	collision_pub.publish( marker );
}

void MotionVisualizer::drawMotion(const size_t& motion_index, const bool& is_best_motion) {
	Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  motion_selector->sampleMotionForDrawing(motion_index, sampling_time_vector, num_samples);

	visualization_msgs::Marker marker_msg;
	marker_msg.header.frame_id = drawing_frame;
	marker_msg.header.stamp = ros::Time::now();
	marker_msg.ns = "path_marker";
	marker_msg.id = motion_index;
	marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
	marker_msg.action = visualization_msgs::Marker::ADD;
	marker_msg.pose.orientation.w = 1.0;
	marker_msg.scale.x = 0.01; 
	marker_msg.lifetime = ros::Duration(1.0);

	if (is_best_motion) {
		// magenta
		marker_msg.color.r = 1.0;
		marker_msg.color.g = 0.0;
		marker_msg.color.b = 1.0;
		marker_msg.color.a = 1.0;
	}
	else {
		// green (0% collision) to red (100% collision)
		marker_msg.color.r = collision_probabilities.at(motion_index);
		marker_msg.color.g = 1.0 - collision_probabilities.at(motion_index);
		marker_msg.color.b = 0.0;
		// higher collision probabilites are less visible
		marker_msg.color.a = 1.0 - 0.5*collision_probabilities.at(motion_index); 
	}

	Vector3 sigma;
	MotionLibrary* motion_library_ptr = motion_selector->GetMotionLibraryPtr();
	for (size_t sample = 0; sample < num_samples; sample++) {
		marker_msg.points.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample), drawing_frame).pose.position);

		sigma = motion_library_ptr->getSigmaAtTime(sampling_time_vector(sample))*(1+0.2*motion_library_ptr->getMotionFromIndex(motion_index).getVelocity(sampling_time_vector(sample)).norm());
		if (is_best_motion) {
			drawGaussianPropagation(sample, sample_points_xyz_over_time.row(sample), sigma);
		}
	}
	drawCollisionIndicator(motion_index, sample_points_xyz_over_time.row(num_samples-1), collision_probabilities.at(motion_index));

 	if (is_best_motion) {
    	best_path_pub.publish(marker_msg);
	}
	else {
		paths_pub.publish(marker_msg);
	}

}

void MotionVisualizer::drawAll() {

	// Clear the previous set of markers
	visualization_msgs::Marker clear_marker_msg;
	clear_marker_msg.header.frame_id = drawing_frame;
	clear_marker_msg.header.stamp = ros::Time::now();
	clear_marker_msg.ns = "path_marker";
	clear_marker_msg.pose.orientation.w = 1.0;
    clear_marker_msg.color.a = 1.0;
    clear_marker_msg.points.clear();
	
    paths_pub.publish(clear_marker_msg);
    best_path_pub.publish(clear_marker_msg);


	size_t num_motions = collision_probabilities.size();
	for (size_t motion_index = 0; motion_index < num_motions; motion_index++) {
		bool is_best_motion = (motion_index == *best_traj_index);
		drawMotion(motion_index, is_best_motion);
	}

}


// Custom sorting functions to get 