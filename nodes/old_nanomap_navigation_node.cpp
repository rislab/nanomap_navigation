#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nanomap_navigation/motion_selector.h>
#include <nanomap_navigation/motion_visualizer.h>
#include <nanomap_navigation/motion_selector_utils.h>

#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;

class NanoMapNavigationNode {
public:

    NanoMapNavigationNode() : nh("~") {
    
        // Initialization
        double acceleration_interpolation_min;
        double soft_top_speed;
        double speed_at_acceleration_max;
        double acceleration_interpolation_max;

        // Get values from param server
        pu::get("acceleration_interpolation_min", acceleration_interpolation_min);
        pu::get("soft_top_speed", soft_top_speed);
        pu::get("speed_at_acceleration_max", speed_at_acceleration_max);
        pu::get("acceleration_interpolation_max", acceleration_interpolation_max);
        pu::get("flight_altitude", flight_altitude);

        this->soft_top_speed_max = soft_top_speed;

        // Using motion_selector only to get the motion_library
        motion_selector.InitializeLibrary(use_3d_library, final_time, soft_top_speed, acceleration_interpolation_min, speed_at_acceleration_max, acceleration_interpolation_max);
        motion_selector.SetNominalFlightAltitude(flight_altitude);

        motion_visualizer.initialize(&motion_selector, nh, &best_traj_index, final_time);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Subscribers
		camera_info_sub = nh.subscribe("/camera/camera_info", 1, &NanoMapNavigationNode::OnCameraInfo, this);
        pose_sub = nh.subscribe("pose_topic", 100, &NanoMapNavigationNode::OnPose, this);
        velocity_sub = nh.subscribe("twist_topic", 100, &NanoMapNavigationNode::OnVelocity, this);
        local_goal_sub = nh.subscribe("local_goal_topic", 1, &NanoMapNavigationNode::OnLocalGoal, this);

        // Publishers
        carrot_pub = nh.advertise<visualization_msgs::Marker>("carrot_marker_topic", 0);
    }

	bool got_camera_info = false;
	void OnCameraInfo(const sensor_msgs::CameraInfo msg) {
		if (got_camera_info) {
			return;
		}
		double height = msg.height;
		double width = msg.width;
		Matrix3 K_camera_info;
		K_camera_info << msg.K[0], msg.K[1], msg.K[2], msg.K[3], msg.K[4], msg.K[5], msg.K[6], msg.K[7], msg.K[8];
		if (msg.binning_x != msg.binning_y) { std::cout << "WARNING: Binning isn't same for camera info" << std::endl;}
		double bin = msg.binning_x;
		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
		if (depth_image_collision_ptr != nullptr) {
			depth_image_collision_ptr->setCameraInfo(bin, width, height, K_camera_info);
			got_camera_info = true;
			ROS_WARN_THROTTLE(1.0, "Received camera info");
		}
        // TODO: JON set the CameraInfo header.frame_id or set this from params
		depth_sensor_frame = msg.header.frame_id;
	}

	void SetThrustForLibrary(double thrust) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setThrust(thrust);
		}
	}

	bool CheckIfInevitableCollision(std::vector<double> const collision_probabilities) {
        // All trajectories have a collision probability >= 0.6
		for (size_t i = 0; i < collision_probabilities.size(); i++) {
			if (collision_probabilities.at(i) < 0.6) {
				return false;
			}
		}
		return true;
	}

	void ReactToSampledPointCloud() {
		if (!got_camera_info && use_depth_image) {
			ROS_WARN_THROTTLE(1.0, "Haven't received camera info yet");
		}

		auto t1 = std::chrono::high_resolution_clock::now();
		motion_selector.computeBestEuclideanMotion(carrot_ortho_body_frame, best_traj_index, desired_acceleration);
		
      	std::vector<double> collision_probabilities = motion_selector.getCollisionProbabilities();
		motion_visualizer.setCollisionProbabilities(collision_probabilities);
		if (executing_e_stop || CheckIfInevitableCollision(collision_probabilities)) {
			ExecuteEStop();
		}
	    else if (yaw_on) {
	    	SetYawFromMotion();
	    } 
	    // auto t2 = std::chrono::high_resolution_clock::now();
	    // std::cout << "ReactToSampledPointCloud took "
     //  		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
     //  		<< " microseconds\n";

		// JON: use_acl (whatever that is) should always be true
		// if (!use_acl){PublishCurrentAttitudeSetpoint();}
	}

	void ExecuteEStop() {
		best_traj_index = 0; // this overwrites the "best acceleration motion"

		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		// If first time entering e stop, compute open loop parameters
		if (!executing_e_stop) {
			begin_e_stop_time = ros::Time::now().toSec();
			if (motion_library_ptr != nullptr) {
				double e_stop_acceleration_magnitude = 9.8*tan(max_e_stop_pitch_degrees * M_PI / 180.0);
				Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.0);
				Vector3 e_stop_acceleration = -1.0 * e_stop_acceleration_magnitude * initial_velocity_ortho_body/initial_velocity_ortho_body.norm();
				motion_library_ptr->setBestAccelerationMotion(e_stop_acceleration);
				Vector3 end_jerk_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.2);

				e_stop_time_needed = end_jerk_velocity_ortho_body.norm() / e_stop_acceleration_magnitude / 0.85;
				ROS_WARN_THROTTLE(1.0, "E-STOPPING");
			}
		}
		executing_e_stop = true;
		if (motion_library_ptr != nullptr) {
			desired_acceleration = motion_library_ptr->getMotionFromIndex(best_traj_index).getAcceleration();
		}


		// Check if time to exit open loop e stop
		double e_stop_time_elapsed = ros::Time::now().toSec() - begin_e_stop_time;
		std::cout << "E STOP TIME ELAPSED " << e_stop_time_elapsed << std::endl;
		if (e_stop_time_elapsed > e_stop_time_needed) {
			executing_e_stop = false;
		}
	}

	void ComputeBestAccelerationMotion() {
		if (executing_e_stop) { //Does not compute if executing e stop
			return;
		}

		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			// compute best acceleration in open field
			double time_to_eval = 0.5;
			Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.0);
			Vector3 position_if_dont_accel = initial_velocity_ortho_body*time_to_eval;
			Vector3 vector_towards_goal = (carrot_ortho_body_frame - position_if_dont_accel);
			Vector3 best_acceleration = ((vector_towards_goal/vector_towards_goal.norm()) * soft_top_speed_max - initial_velocity_ortho_body) / time_to_eval;
			double current_max_acceleration = motion_library_ptr->getNewMaxAcceleration();
			if (best_acceleration.norm() > current_max_acceleration) {
				best_acceleration = best_acceleration * current_max_acceleration / best_acceleration.norm();
			}
			motion_library_ptr->setBestAccelerationMotion(best_acceleration);

			// if within stopping distance, line search for best stopping acceleration
			Vector3 stop_position = motion_library_ptr->getMotionFromIndex(0).getTerminalStopPosition(0.5);
			double stop_distance = stop_position.dot(vector_towards_goal/vector_towards_goal.norm());
			double distance_to_carrot = carrot_ortho_body_frame(0);
			
			int max_line_searches = 10;
			int counter_line_searches = 0;
			while ( (stop_distance > distance_to_carrot) && (counter_line_searches < max_line_searches) ) {
				best_acceleration = best_acceleration * distance_to_carrot / stop_distance;
				if (best_acceleration.norm() > current_max_acceleration) {
					best_acceleration = best_acceleration * current_max_acceleration / best_acceleration.norm();
				}
				motion_library_ptr->setBestAccelerationMotion(best_acceleration);
				stop_position = motion_library_ptr->getMotionFromIndex(0).getTerminalStopPosition(0.5);
				stop_distance = stop_position.dot(vector_towards_goal/vector_towards_goal.norm());
				counter_line_searches++;	
			} 
		}
	}

    void PublishCurrentAttitudeSetpoint() {
        double forward_propagation_time = ros::Time::now().toSec() - last_pose_update.toSec();
        Vector3 attitude_thrust_desired = attitude_generator.generateDesiredAttitudeThrust(desired_acceleration, forward_propagation_time);
        SetThrustForLibrary(attitude_thrust_desired(2));

        PassToOuterLoop(desired_acceleration);
        if (use_3d_library) {
            AltitudeFeedbackOnBestMotion();
        }
		// if (!use_acl){PublishAttitudeSetpoint(attitude_thrust_desired);}
    }

	void AltitudeFeedbackOnBestMotion() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
				Motion best_motion = motion_library_ptr->getMotionFromIndex(best_traj_index);
				Vector3 best_motion_position_ortho_body =  best_motion.getPosition(1.0);
				Vector3 best_motion_position_world = TransformOrthoBodyToWorld(best_motion_position_ortho_body);
				double new_z_setpoint = best_motion_position_world(2);
				attitude_generator.setZsetpoint(new_z_setpoint);
		}
	}

	bool UseDepthImage() {
		return use_depth_image;
	}

    void drawAll() {
        motion_visualizer.drawAll();
    }

private:

    void PassToOuterLoop(Vector3 desired_acceleration_setpoint) {
        if (!motion_primitives_live) {return;}

        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {

            Motion best_motion = motion_library_ptr->getMotionFromIndex(best_traj_index);

			// TODO: (JON) update this with control arch
            // build up QuadGoal
            acl_fsw::QuadGoal quad_goal;
            quad_goal.cut_power = false;
            quad_goal.xy_mode = acl_fsw::QuadGoal::MODE_ACCEL;
            quad_goal.z_mode = acl_fsw::QuadGoal::MODE_POS;

            Vector3 pos = TransformOrthoBodyToWorld(best_motion.getPosition(0.5));
            Vector3 vel = RotateOrthoBodyToWorld(best_motion.getVelocity(0.5));
            Vector3 accel = RotateOrthoBodyToWorld(best_motion.getAcceleration());
            Vector3 jerk = RotateOrthoBodyToWorld(best_motion.getJerk());
				
            quad_goal.jerk.x = jerk(0);
            quad_goal.jerk.y = jerk(1);
            quad_goal.jerk.z = jerk(2);
            quad_goal.accel.x = accel(0);
            quad_goal.accel.y = accel(1);
            quad_goal.accel.z = accel(2);
            //quad_goal.vel.x = vel(0);
            //quad_goal.vel.y = vel(1);
            //quad_goal.pos.x = pos(0);
            //quad_goal.pos.y = pos(1);
            if (use_3d_library) {
                quad_goal.pos.z = pos(2);
                quad_goal.vel.z = vel(2);
            } else {
                double vel = 0;
                double accel = 0;
                double dolphin_altitude = DolphinStrokeDetermineAltitude(speed, vel, accel);
                carrot_world_frame(2) = dolphin_altitude; 
                quad_goal.pos.z = dolphin_altitude;
                quad_goal.vel.z = vel;
                quad_goal.accel.z = accel;
            }

            UpdateYaw();
            quad_goal.yaw = -set_bearing_azimuth_degrees*M_PI/180.0;

            if (stationary_yawing) {
                quad_goal.xy_mode = acl_fsw::QuadGoal::MODE_VEL;
                quad_goal.jerk.x = 0;
                quad_goal.jerk.y = 0;
                quad_goal.vel.x = 0;
                quad_goal.vel.y = 0;
            }

            quad_goal_pub.publish(quad_goal);
        }
    }

	// TODO: (JON) publish the go command messsage
    bool motion_primitives_live = false;
    void OnCommand(const fla_msgs::FlightCommand& msg)  {
        if (msg.command == fla_msgs::FlightCommand::CMD_GO){
            ROS_INFO("GOT GO COMMAND");
            motion_primitives_live = true;

            ROS_INFO("Starting");	
        }
        else{
            motion_primitives_live = false;
        }	
    }

    bool stationary_yawing = false;
    void SetYawFromMotion() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			// get position at t=0
			Vector3 initial_position_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getPosition(0.0);
			// get velocity at t=0
			Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.5);
			Vector3 final_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.5);
			// normalize velocity
			double speed_initial = initial_velocity_ortho_body.norm();
			double speed_final = final_velocity_ortho_body.norm();
			if (speed_final != 0) {
				final_velocity_ortho_body = final_velocity_ortho_body / speed_final;
			}

			// add normalized velocity to position to get a future position
			Vector3 final_position_ortho_body = initial_position_ortho_body + initial_velocity_ortho_body;
			// yaw towards future position using below

			Vector3 final_position_world = TransformOrthoBodyToWorld(final_position_ortho_body);
			
			// if already going slow, and carrot close, then don't yaw (otherwise we spin around and hunt)
			if (speed_initial < 2.0 && carrot_ortho_body_frame.norm() < 1.0) {
				stationary_yawing = false;
				motion_selector.SetSoftTopSpeed(soft_top_speed_max);
				return;
			}


			if ((final_position_world(0) - pose_global_x)!= 0) {
				double potential_bearing_azimuth_degrees = CalculateYawFromPosition(final_position_world);
				double actual_bearing_azimuth_degrees = -pose_global_yaw * 180.0/M_PI;
				double bearing_error = potential_bearing_azimuth_degrees - actual_bearing_azimuth_degrees;
				while(bearing_error > 180) { 
					bearing_error -= 360;
				}
				while(bearing_error < -180) { 
					bearing_error += 360;
				}

				// if inside 100 degrees, let it yaw towards
				if (abs(bearing_error) < 50.0)  {
					stationary_yawing = false;
					motion_selector.SetSoftTopSpeed(soft_top_speed_max);
					bearing_azimuth_degrees = potential_bearing_azimuth_degrees;
					return;
				}

				// else, slow down
				motion_selector.SetSoftTopSpeed(0.1);
				stationary_yawing = false;
				if (speed_initial < 0.5) {
					bearing_azimuth_degrees = CalculateYawFromPosition(carrot_world_frame);
					stationary_yawing = true;
				}

			}
		}
	}

    ros::Time last_pose_update;
    void OnPose(geometry_msgs::PoseWithCovarianceStamped const& pose_covariance) {

        geometry_msgs::PoseStamped pose;
        pose.header = pose_covariance.header;
        pose.pose = pose_covariance.pose.pose;

        // if ((ros::Time::now() - last_point_cloud_received).toSec() > 0.1) {
        // 	ReactToSampledPointCloud();
        // }

        motion_selector.UpdateCurrentAltitude(pose.pose.position.z);

        tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        last_pose_update = pose.header.stamp;
        UpdateMotionLibraryRollPitch(roll, pitch);

        // TODO add selection back in
        // ComputeBestAccelerationMotion();
        SetPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);

        // TODO add nanomap stuff back here
    }

    void UpdateMotionLibraryRollPitch(double roll, double pitch) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {
            motion_library_ptr->setRollPitch(roll, pitch);
        }
    }

    void OnVelocity(geometry_msgs::TwistWithCovarianceStamped const& twist_covariance) {
        geometry_msgs::TwistStamped twist;
        twist.header = twist_covariance.header;
        twist.twist = twist_covariance.twist.twist;

        // // attitude_generator.setZvelocity(twist.twist.linear.z);
        Vector3 velocity_body_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
        Vector3 velocity_ortho_body_frame = TransformBodyToOrthoBody(velocity_body_frame);
        if (!use_3d_library) {
            velocity_ortho_body_frame(2) = 0.0;  // WARNING for 2D only
        }
        UpdateMotionLibraryVelocity(velocity_ortho_body_frame);
        speed = velocity_ortho_body_frame.norm();
        //UpdateTimeHorizon(speed);
        UpdateMaxAcceleration(speed);
    }

    void OnLocalGoal(geometry_msgs::PoseStamped const& local_goal) {
        carrot_world_frame(0) = local_goal.pose.position.x; 
        carrot_world_frame(1) = local_goal.pose.position.y;
        carrot_world_frame(2) = local_goal.pose.position.z;
        UpdateCarrotOrthoBodyFrame();
        visualization_msgs::Marker marker;
        marker.header.frame_id = "ortho_body";
        marker.header.stamp = ros::Time::now();
        marker.ns = "carrot_namespace";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = carrot_ortho_body_frame(0);
        marker.pose.position.y = carrot_ortho_body_frame(1);
        marker.pose.position.z = carrot_ortho_body_frame(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 0.5;
        marker.color.r = 0.9;
        marker.color.g = 0.4;
        marker.color.b = 0.0;
        carrot_pub.publish(marker);
    }

    void SetPose(double x, double y, double z, double yaw) {
        pose_global_x = x;
        pose_global_y = y;
        pose_global_z = z;
        pose_global_yaw = yaw;
    }
 
    Vector3 TransformBodyToOrthoBody(Vector3 const& body_frame) {
        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("ortho_body", "rocky0704/base", 
                                            ros::Time(0), ros::Duration(1/30.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("ID 6 %s", ex.what());
            return Vector3(1,1,1);
        }

        Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        Matrix3 R = quat.toRotationMatrix();
        return R*body_frame;
    }

	void UpdateMotionLibraryVelocity(Vector3 const& velocity_ortho_body_frame) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {
            motion_library_ptr->setInitialVelocity(velocity_ortho_body_frame);
        }
	}

    void UpdateMaxAcceleration(double speed) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
            if (motion_library_ptr != nullptr) {
                motion_library_ptr->UpdateMaxAcceleration(speed);
            }
    }

    void UpdateCarrotOrthoBodyFrame() {
        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("ortho_body", "world", 
                                            ros::Time(0), ros::Duration(1.0/30.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("ID 2 %s", ex.what());
            return;
        }

        geometry_msgs::PoseStamped pose_global_goal_world_frame = PoseFromVector3(carrot_world_frame, "world");
        geometry_msgs::PoseStamped pose_global_goal_ortho_body_frame = PoseFromVector3(Vector3(0,0,0), "ortho_body");

        tf2::doTransform(pose_global_goal_world_frame, pose_global_goal_ortho_body_frame, tf);
        carrot_ortho_body_frame = VectorFromPose(pose_global_goal_ortho_body_frame);
    }

    ros::Subscriber camera_info_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber local_goal_sub;

	ros::Publisher carrot_pub;

    // TODO: find what depth frame is called
	std::string depth_sensor_frame = "depth_sensor";
  
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2_ros::Buffer tf_buffer_;

    double start_time = 0.0;
    double final_time = 1.5;

	double bearing_azimuth_degrees = 0.0;
	double set_bearing_azimuth_degrees = 0.0;

	Eigen::Vector4d pose_x_y_z_yaw;


	Vector3 carrot_world_frame;
	Vector3 carrot_ortho_body_frame = Vector3(0,0,0);

	size_t best_traj_index = 0;
	Vector3 desired_acceleration = Vector3(0,0,0);

    MotionSelector motion_selector;

	double pose_global_x = 0;
	double pose_global_y = 0;
	double pose_global_z = 0;
	double pose_global_yaw = 0;

	bool yaw_on = false;
    double soft_top_speed_max = 0.0;
    bool use_depth_image = true;
    bool use_3d_library = true;
    double flight_altitude = 1.0;

	bool executing_e_stop = false;
	// double begin_e_stop_time = 0.0;
	// double e_stop_time_needed = 0.0;
	// double max_e_stop_pitch_degrees = 60.0;

    double speed = 0.0;

    ros::NodeHandle nh;

public:
    MotionVisualizer motion_visualizer;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "nanomap_navigation");

    NanoMapNavigationNode nav_node;

    ros::Rate spin_rate(100);
    
    // Draw motion primitives periodically
    size_t counter = 0;
    while (ros::ok()) {
        counter++;
        if (counter > 3) {
            counter = 0;
            nav_node.drawAll();
        }
        
        ros::spinOnce();
        spin_rate.sleep();
    }

    return 0;
}