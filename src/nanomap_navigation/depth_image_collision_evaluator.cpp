#include <nanomap_navigation/depth_image_collision_evaluator.h>

#define num_nearest_neighbors 1

void DepthImageCollisionEvaluator::UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
	xyz_cloud_ptr = xyz_cloud_new;
  my_kd_tree_depth_image.Initialize(xyz_cloud_ptr);
}

void DepthImageCollisionEvaluator::UpdateLaserPointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
  xyz_laser_cloud_ptr = xyz_cloud_new;
  my_kd_tree_laser.Initialize(xyz_laser_cloud_ptr);
}

void DepthImageCollisionEvaluator::UpdateRotationMatrix(Matrix3 const R) {
  this->R = R;
};

void DepthImageCollisionEvaluator::UpdateBodyToRdf(Matrix3 const& R) {
  R_body_to_rdf = R;
  R_body_to_rdf_inverse = R.inverse();
}

bool DepthImageCollisionEvaluator::computeDeterministicCollisionOnePositionKDTree(Vector3 const& robot_position) {
  if (robot_position(2) < -1.0) {
    return true;
  }
  my_kd_tree_depth_image.SearchForNearest<1>(robot_position[0], robot_position[1], robot_position[2]);
  if (my_kd_tree_depth_image.squared_distances.size() > 0) {
    if (my_kd_tree_depth_image.squared_distances[0] < 2.0) {
      return true;
    }
  }
  return false;
}

double ThresholdSigmoid(double value) {
    double sigmoid_threshold = 0.99;
    if (value > sigmoid_threshold) { 
      double sigmoid = 1.0 / (1.0 + exp(-value));
      return sigmoid_threshold + (1 - sigmoid_threshold) * sigmoid;
    }
    if (value < 0.0) { 
      return 0.0;
    }
    return value;
}

double ThresholdHard(double value) {
    double hard_threshold = 0.9;
    if (value > hard_threshold) { 
      return hard_threshold;
    }
    if (value < 0.0) { 
      return 0.0;
    }
    return value;
}

bool DepthImageCollisionEvaluator::IsBehind(Vector3 robot_position) {
    return (robot_position(2) < -0.5);
}

bool DepthImageCollisionEvaluator::IsOutsideDeadBand(Vector3 robot_position) {
    return (robot_position.squaredNorm() > 0.5);
}

double DepthImageCollisionEvaluator::IsOutsideFOV(Vector3 robot_position) {
    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    // Checks if outside left/right FOV
    if ( (pi_x < 0) || (pi_x > (num_x_pixels - 1)) ) {
        return p_collision_left_right_fov;
    }
    // Checks if above top/bottom FOV
    if (pi_y < 0) {
      return p_collision_up_down_fov; 
    }
    if (pi_y > (num_y_pixels - 1)) {
      return p_collision_up_down_fov; 
    }

    //Checks for occlusion
    if (xyz_cloud_ptr == nullptr) {
      return 0.0;
    } 
    pcl::PointXYZ point = xyz_cloud_ptr->at(pi_x,pi_y);
    if (std::isnan(point.z)) { 
       return 0.0;
    }
    Vector3 position_ortho_body = Vector3(point.x, point.y, point.z);
    Vector3 position_rdf = R * position_ortho_body;
    if( robot_position(2) >  position_rdf(2) ) {
      //std::cout << "OCCLUSION" << std::endl;
      return p_collision_occluded;
    }
    return 0.0;
}

double DepthImageCollisionEvaluator::AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision) {
    if (IsBehind(robot_position)) {
      return ThresholdSigmoid(probability_of_collision + p_collision_behind);
    }
    if (IsOutsideDeadBand(robot_position)) {
      return ThresholdSigmoid(probability_of_collision + IsOutsideFOV(robot_position));
    }
    return ThresholdSigmoid(probability_of_collision);
}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree_DepthImage(Vector3 const& robot_position, Vector3 const& sigma_robot_position, bool early_exit) {
  double probability_of_collision = 0.0;
  if (1) {

    NanoMapKnnArgs args;
    args.query_point_current_body_frame = R_body_to_rdf_inverse*R*robot_position;
    args.axis_aligned_linear_covariance = R_body_to_rdf_inverse*R*sigma_robot_position;
    args.early_exit = early_exit;
    NanoMapKnnReply reply = nanomap.KnnQuery(args);

    if (NANOMAP_DEBUG_PRINT) {
      std::cout << "args.query_point_current_body_frame" << args.query_point_current_body_frame.transpose() << std::endl;
      std::cout << "args.axis_aligned_linear_covariance" << args.axis_aligned_linear_covariance.transpose() << std::endl;

      std::cout << "frame_id "   << reply.frame_id   << std::endl;
      std::cout << "fov_status " << reply.fov_status << std::endl;
      std::cout << "query_point_in_frame_id " << reply.query_point_in_frame_id.transpose() << std::endl;
      std::cout << "closest_points_in_frame_id.size() " << reply.closest_points_in_frame_id.size() << std::endl;
      std::cout << "axis_aligned_linear_covariance" << reply.axis_aligned_linear_covariance.transpose() << std::endl;
      if ((reply.closest_points_in_frame_id.size() > 0) && (my_kd_tree_depth_image.closest_pts.size() > 0)) {
        std::cout << "NanoMap      distance_between           " << (reply.closest_points_in_frame_id[0] - reply.query_point_in_frame_id).norm() << std::endl;
        std::cout << "             closest_points_in_frame_id " << reply.closest_points_in_frame_id[0].transpose() << std::endl;
        pcl::PointXYZ point = my_kd_tree_depth_image.closest_pts[0];
        std::cout << "old school   distance_between           " << (robot_position - Vector3(point.x, point.y, point.z)).norm() << std::endl;
        std::cout << "             closest_pts                " << Vector3(point.x, point.y, point.z).transpose() << std::endl;
        
      }
      std::cout << std::endl; 
    }

    std::vector<pcl::PointXYZ> pcl_vector;
    for (int i =0; i < reply.closest_points_in_frame_id.size(); i++) {
      pcl::PointXYZ point;
      point.x = reply.closest_points_in_frame_id[i](0);
      point.y = reply.closest_points_in_frame_id[i](1);
      point.z = reply.closest_points_in_frame_id[i](2);
      pcl_vector.push_back(point);
    }

    if (reply.fov_status != NanoMapFovStatus::not_initialized) {
      probability_of_collision = computeProbabilityOfCollisionNPositionsKDTree(reply.query_point_in_frame_id, reply.axis_aligned_linear_covariance, pcl_vector, 0.8);  
    }
    else {
      probability_of_collision = 0.0;
    }
    
    if (reply.fov_status == NanoMapFovStatus::behind) {
      return ThresholdSigmoid(probability_of_collision + p_collision_behind);
    }
    if (reply.fov_status == NanoMapFovStatus::beyond_sensor_horizon) {
      return ThresholdSigmoid(probability_of_collision + p_collision_beyond);
    }
    if (reply.fov_status == NanoMapFovStatus::laterally_outside_fov) {
      return ThresholdSigmoid(probability_of_collision + p_collision_left_right_fov);
    }
    if (reply.fov_status == NanoMapFovStatus::occluded) {
      return ThresholdSigmoid(probability_of_collision + p_collision_occluded);
    }
  }

  return ThresholdSigmoid(probability_of_collision);
}


double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree_Laser(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
  if (xyz_laser_cloud_ptr != nullptr) {
    my_kd_tree_laser.SearchForNearest<num_nearest_neighbors>(robot_position[0], robot_position[1], robot_position[2]);
    double probability_of_collision = computeProbabilityOfCollisionNPositionsKDTree(robot_position, sigma_robot_position, my_kd_tree_laser.closest_pts, 0.3);
    return ThresholdHard(probability_of_collision);
  }
  return 0.0;
}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position, std::vector<pcl::PointXYZ> const& closest_pts, double interpolation_radius) {
  double probability_no_collision = 1.0;

  if (closest_pts.size() > 0) {
    for (size_t i = 0; i < std::min((int)closest_pts.size(), num_nearest_neighbors); i++) {

      pcl::PointXYZ first_point = closest_pts[i];
      Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);

      // interpolate towards robot
      double norm = (robot_position - depth_position).norm();
      if (norm >= 2*interpolation_radius) {
        depth_position = depth_position + (robot_position - depth_position)/norm*interpolation_radius;
      } else {
        depth_position = depth_position + (robot_position - depth_position)*0.5;
      }

      Vector3 sigma_robot_position_nn = sigma_robot_position*0.0;
      for (int i = 0; i <3; i++) {
        if (sigma_robot_position_nn(i) < 0) {
          sigma_robot_position_nn(i)=-sigma_robot_position_nn(i);
        }
      }
      Vector3 total_sigma = sigma_robot_position_nn + sigma_depth_point;
      Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));  
    
      double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
      double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
      double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

      double probability_of_collision = volume / denominator * pow(2.71828,exponent);

      probability_no_collision = probability_no_collision * (1 - probability_of_collision);
    }
    return 1 - probability_no_collision;
  }
  return 0.0; // if no points in closest_pts
}

void DepthImageCollisionEvaluator::setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info) {
  nanomap.SetCameraInfo(bin, width, height, K_camera_info);

  if (bin < 1.0) {binning = 1.0;}
  else {binning = bin;}
  num_x_pixels = width / binning;
  num_y_pixels = height / binning;
  K = K_camera_info;
  K /= binning;
  K(2,2) = 1.0;
  return;
}

