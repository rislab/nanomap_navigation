/***
 * From motion_selector_node.cpp
 * Broken parts commented out
 * Disabled parts commented out
    * E.g. all "Hokuyo laser" references commented out
***/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
// #include "fla_msgs/ProcessStatus.h"
// #include "fla_msgs/FlightCommand.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>

// #include "acl_fsw/QuadGoal.h"

#include <nanomap_navigation/motion_selector.h>
#include <nanomap_navigation/attitude_generator.h>
#include <nanomap_navigation/motion_visualizer.h>
#include <nanomap_ros/nanomap_visualizer.h>

// #include "fla_utils/param_utils.h"
#include <parameter_utils/ParameterUtils.h>
#include <control_arch/Waypoints.h>
#include <control_arch/utils/state_t.h>
#include <control_arch/trajectory/Waypoints.h>
#include <control_arch/Waypoints.h>

#define GRAVITY_CONSTANT 9.8

namespace pu = parameter_utils;
namespace gu = geometry_utils;

class NanoMapNavigationNode {
public:

    NanoMapNavigationNode() : nh("~") {

        // Initialization
        double acceleration_interpolation_min;
        double soft_top_speed;
        double speed_at_acceleration_max;
        double acceleration_interpolation_max;
        double offset;
        double sensor_range;
        int N_depth_image_history;

        // Changed from fla_utils::SafeGetParam() to pu::get()
        pu::get("soft_top_speed", soft_top_speed);
        pu::get("acceleration_interpolation_min", acceleration_interpolation_min);
        pu::get("yaw_on", yaw_on);
        pu::get("use_depth_image", use_depth_image);
        pu::get("speed_at_acceleration_max", speed_at_acceleration_max);
        pu::get("acceleration_interpolation_max", acceleration_interpolation_max);
        pu::get("flight_altitude", flight_altitude);
        pu::get("use_3d_library", use_3d_library);
        pu::get("use_acl", use_acl);
        pu::get("max_e_stop_pitch_degrees", max_e_stop_pitch_degrees);
        // pu::get("laser_z_below_project_up", laser_z_below_project_up);
        pu::get("sensor_range", sensor_range);
        // pu::get("A_dolphin", A_dolphin);
        // pu::get("T_dolphin", T_dolphin);
        // pu::get("use_lidar_lite_z", use_lidar_lite_z);
        pu::get("thrust_offset", offset);
        pu::get("N_depth_image_history", N_depth_image_history);
        pu::get("body_frame_id", body_frame_id);

        this->soft_top_speed_max = soft_top_speed;
        default_body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;

        motion_selector.InitializeLibrary(use_3d_library, final_time, soft_top_speed, acceleration_interpolation_min, speed_at_acceleration_max, acceleration_interpolation_max);
        motion_selector.SetNominalFlightAltitude(flight_altitude);
        attitude_generator.setZsetpoint(flight_altitude);
        attitude_generator.setOffset(offset);
        DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
        if (depth_image_collision_ptr != nullptr) {
            depth_image_collision_ptr->nanomap.SetNumDepthImageHistory(N_depth_image_history);
            depth_image_collision_ptr->nanomap.SetSensorRange(sensor_range);
        }

        motion_visualizer.initialize(&motion_selector, nh, &best_traj_index, final_time);
        nanomap_visualizer.Initialize(nh);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        std::cout << "Waiting for transforms" << std::endl;
        WaitForTransforms("world", "ortho_body"); 
        std::cout << "Received transform" << std::endl;
        // last_pose_update = ros::Time::now();
        // PublishOrthoBodyTransform(0.0, 0.0); // initializes ortho_body transform to be with 0, 0 roll, pitch

        // wait until the we are in hover (otherwise we execute E-STOP immediately)
        // TODO: remove this and enable flight_command
        ros::Duration(20.0).sleep();

        // Subscribers
        odom_sub = nh.subscribe("odometry_topic", 100, &NanoMapNavigationNode::OnOdometry, this);
        // pose_sub = nh.subscribe("pose_topic", 100, &NanoMapNavigationNode::OnPose, this);
        // velocity_sub = nh.subscribe("twist_topic", 100, &NanoMapNavigationNode::OnVelocity, this);
        
        camera_info_sub = nh.subscribe("depth_camera_info_topic", 1, &NanoMapNavigationNode::OnCameraInfo, this);
        depth_image_sub = nh.subscribe("depth_camera_pointcloud_topic", 1, &NanoMapNavigationNode::OnDepthImage, this);
        
        // max_speed_sub = nh.subscribe("/max_speed", 1, &NanoMapNavigationNode::OnMaxSpeed, this);
        local_goal_sub = nh.subscribe("local_goal_topic", 1, &NanoMapNavigationNode::OnLocalGoal, this);
        // laser_scan_sub = nh.subscribe("laser_scan_topic", 1, &NanoMapNavigationNode::OnScan, this);
        smoothed_pose_sub = nh.subscribe("/samros/keyposes", 100, &NanoMapNavigationNode::OnSmoothedPoses, this);
        // height_above_ground_sub = nh.subscribe("/lidarlite_filter/height_above_ground", 1, &NanoMapNavigationNode::OnLidarlite, this);
        command_sub = nh.subscribe("flight_command_topic", 1, &NanoMapNavigationNode::OnCommand, this);

        // Publishers
        carrot_pub = nh.advertise<visualization_msgs::Marker>( "carrot_marker_topic", 0 );
        gaussian_pub = nh.advertise<visualization_msgs::Marker>( "gaussian_visualization_topic", 0 );
        attitude_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("attitude_setpoint_topic", 1);
        attitude_setpoint_visualization_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_visualization_topic", 1);
        // status_pub = nh.advertise<fla_msgs::ProcessStatus>("status_topic", 0);
        // quad_goal_pub = nh.advertise<acl_fsw::QuadGoal>("/FLA_ACL02/goal", 1);
        waypoints_pub = nh.advertise<control_arch::Waypoints>("trajectory/waypoints", 1);
     }

     void WaitForTransforms(std::string first, std::string second) {
         for(;;){
            try {
              tf_buffer_.lookupTransform(first, second, 
                                            ros::Time(0), ros::Duration(30.0));
            } catch (tf2::TransformException &ex) {
              continue;
            }
            break;
        }
     }

    // void OnMaxSpeed(const std_msgs::Float64 msg) {
    //     soft_top_speed_max = msg.data;
    // }

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
        depth_sensor_frame_id = msg.header.frame_id;
    }

    void SetThrustForLibrary(double thrust) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {
            motion_library_ptr->setThrust(thrust);
        }
    }

    geometry_msgs::TransformStamped GetTransformToWorld() {
        geometry_msgs::TransformStamped tf;
        try {

          tf = tf_buffer_.lookupTransform("world", "ortho_body", 
                                        ros::Time(0), ros::Duration(1.0/30.0));
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("ID 1 %s", ex.what());
          return tf;
        }
        return tf;
    }

    // JON: uses collision_probabilities instead of hokuyo_collision_probabilities
    bool CheckIfInevitableCollision(std::vector<double> const collision_probabilities) {
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
            return;
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        motion_selector.computeBestEuclideanMotion(carrot_ortho_body_frame, best_traj_index, desired_acceleration);
        
          std::vector<double> collision_probabilities = motion_selector.getCollisionProbabilities();
        //   std::vector<double> hokuyo_collision_probabilities = motion_selector.getHokuyoCollisionProbabilities();
        motion_visualizer.setCollisionProbabilities(collision_probabilities);
        // JON: CheckIfInevitableCollision now uses collision_probabilities instead of hokuyo_collision_probabilities
        if (executing_e_stop || CheckIfInevitableCollision(collision_probabilities)) {
            ROS_WARN_THROTTLE(1.0, "Executing E-STOP maneuver");
            ExecuteEStop();
        }
        else if (yaw_on) {
            SetYawFromMotion();
        } 
        // auto t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "ReactToSampledPointCloud took "
     //  		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
     //  		<< " microseconds\n";

        if (!use_acl){PublishCurrentAttitudeSetpoint();}
    }

    void ExecuteEStop() {
        best_traj_index = 0; // this overwrites the "best acceleration motion"

        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        // If first time entering e stop, compute open loop parameters
        if (!executing_e_stop) {
            begin_e_stop_time = ros::Time::now().toSec();
            if (motion_library_ptr != nullptr) {
                double e_stop_acceleration_magnitude = GRAVITY_CONSTANT*tan(max_e_stop_pitch_degrees * M_PI / 180.0);
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
        //std::cout << "E STOP TIME ELAPSED " << e_stop_time_elapsed << std::endl;
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

            bool best_acceleration_is_nan = std::isnan(best_acceleration(0)) || std::isnan(best_acceleration(1)) || std::isnan(best_acceleration(2));
            if (best_acceleration_is_nan) {
                ROS_WARN("best_acceleration in ComputeBestAccelerationMotion is NaN");
                motion_library_ptr->getMotionFromIndex(best_traj_index).printMotionAttributes();
                std::cout << "  carrot_ortho_body_frame: " << carrot_ortho_body_frame.transpose() << std::endl;
                std::cout << "  initial_velocity_ortho_body: " << initial_velocity_ortho_body.transpose() << std::endl;
                std::cout << "  position_if_dont_accel: " << position_if_dont_accel.transpose() << std::endl;
                std::cout << "  vector_towards_goal: " << vector_towards_goal.transpose() << std::endl;
                std::cout << "  best_acceleration: " << best_acceleration.transpose() << std::endl;
                std::cout << "  current_max_acceleration: " << current_max_acceleration << std::endl;
                std::cout << "  best_acceleration.norm(): " << best_acceleration.norm() << std::endl;
                best_acceleration << 1.0, 1.0, 1.0;
            }

            if (best_acceleration.norm() > current_max_acceleration) {
                best_acceleration = best_acceleration * current_max_acceleration / best_acceleration.norm();

                best_acceleration_is_nan = std::isnan(best_acceleration(0)) || std::isnan(best_acceleration(1)) || std::isnan(best_acceleration(2));
                if (best_acceleration_is_nan) {
                    std::cout << "  IF CONDITION best_acceleration: " << best_acceleration.transpose() << std::endl;
                    best_acceleration << 1.0, 1.0, 1.0;
                }
            }
            // TODO: JON Remove this 
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
        // std::cout << "In PublishCurrentAttitudeSetpoint" << std::endl;
        double forward_propagation_time = ros::Time::now().toSec() - last_pose_update.toSec();
        Vector3 attitude_thrust_desired = attitude_generator.generateDesiredAttitudeThrust(desired_acceleration, forward_propagation_time);
        SetThrustForLibrary(attitude_thrust_desired(2));

        if (use_acl){
            // std::cout << "## Entering PassToOuterLoop" << std::endl;
            PassToOuterLoop(desired_acceleration);
        }
        else if (use_3d_library){AltitudeFeedbackOnBestMotion();}
        if (!use_acl){PublishAttitudeSetpoint(attitude_thrust_desired);}
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
        if (false) { // turns off nanomap visualization
            return;
        }
        DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
        if (depth_image_collision_ptr != nullptr) {
            std::vector<Matrix4> edges = depth_image_collision_ptr->nanomap.GetCurrentEdges();
            nanomap_visualizer.DrawFrustums(edges);
        }
    }

private:

    state_t getPoseAtTime(const Motion& motion, const double& t, const double& global_start_time) {
        state_t state;
        
        // Start time
        // state.t = global_start_time + t;
        state.t = t;

        // implicitly converting from Eigen Vector3 to geometry_utils::Vec3
        state.pos = TransformOrthoBodyToWorld(motion.getPosition(t));
        state.vel = RotateOrthoBodyToWorld(motion.getVelocity(t));
        Vector3 acc_des = RotateOrthoBodyToWorld(motion.getAccelerationAtTime(t));
        Vector3 jerk_ref = RotateOrthoBodyToWorld(motion.getJerkAtTime(t));
        // Vector3 acc_des = RotateOrthoBodyToWorld(motion.getAcceleration()); // Pete's constant way: getAcceleration()
        // Vector3 jerk_ref = RotateOrthoBodyToWorld(motion.getJerk());        // Pete's constant way: getJerk()
        state.acc = acc_des; 
        state.jerk = jerk_ref; 
        state.snap.zeros();

        // build rotation matrix from
        Vector3 roll_pitch_thrust = attitude_generator.generateDesiredAttitudeThrust(acc_des, t);
        double pitch_ref = roll_pitch_thrust(1);
        // TODO: JON confirm these are negative
        double roll_ref = -roll_pitch_thrust(0); 
        double yaw_ref = -set_bearing_azimuth_degrees*M_PI/180.0;

        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(yaw_ref, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch_ref, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll_ref, Eigen::Vector3d::UnitX());
        gu::Rot3d rot3d(rot);
        state.rot = rot3d;
        
        // Differential flatness equations for angular velocities
        Vector3 x_c = Vector3(cos(yaw_ref), sin(yaw_ref), 0.0);
        Vector3 y_c = Vector3(sin(yaw_ref), cos(yaw_ref), 0.0);
        Vector3 z_bdes = acc_des.normalized();
        Vector3 x_bdes = (y_c.cross(z_bdes)).normalized();
        Vector3 y_bdes = z_bdes.cross(x_bdes);
        double c = z_bdes.dot(acc_des); // component of desired acceleration in z body
        double dyaw_ref = 0.0;

        double wx = (-y_bdes.dot(jerk_ref)) / c;
        double wy = (y_bdes.dot(jerk_ref)) / c;
        double wz = (dyaw_ref*x_c.dot(x_bdes) + wy*y_c.dot(z_bdes))
                    / (y_c.cross(z_bdes)).norm();
        state.ang = gu::Vec3(wx, wy, wz); // TODO: does Pete implicitly just do .zeros() here?
        state.angacc.zeros();

        state.dyaw = 0.0;
        state.d2yaw = 0.0;
        state.d3yaw = 0.0;

        return state;
    }

    std::vector<state_t> samplePath(const Motion& motion, const double& dt, 
    const double& tstart, const double& tfinal, const double& global_start_time) {
        std::vector<state_t> traj_path;

        for (double t = tstart; t <= tfinal + 1e-6; t+=dt)
            traj_path.push_back(getPoseAtTime(motion, t, global_start_time));

        return traj_path;
    }

    void PassToOuterLoop(Vector3 desired_acceleration_setpoint) {
        if (!motion_primitives_live) {return;}

        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {

            Motion best_motion = motion_library_ptr->getMotionFromIndex(best_traj_index);

            UpdateYaw(); // TODO (JON): Is this necessary here?

            // TODO: JON set these from parameter server
            double duration = 1.0;
            double dt = 0.005;
            // double start = 0.01;
            // double end = start + dt - 0.0001;
            double global_start_time = ros::Time::now().toSec();

            // Returns sampled path in world frame
            std::vector<state_t> wpts = samplePath(best_motion, dt, 0.0, duration, global_start_time);
            
            // Generate a waypoints message and schedule the primitive
            control_arch::Waypoints waypoints_msg;
            // Waypoints traj(wpts, dt);
            Waypoints traj(wpts);
            traj.toMessage(waypoints_msg);

            waypoints_msg.start_time = ros::Time::now();
            waypoints_msg.duration = ros::Duration(duration);
            waypoints_msg.interval = dt;

            waypoints_msg.header.stamp = ros::Time::now();
            waypoints_msg.header.frame_id = "world";

            waypoints_msg.trajectory_options.scheduling_mode =
                control_arch::TrajectoryOptions::REPLACE;
            waypoints_msg.trajectory_options.enable_on_ramp = false;
            waypoints_msg.trajectory_options.enable_off_ramp = false;
            waypoints_msg.trajectory_options.required_flag = "hover"; 

            waypoints_pub.publish(waypoints_msg);
            // std::cout << "publishing " << wpts.size() << " waypoints" << std::endl;

            // std::cout 
            //     << "pos: \n"
            //     << pos(0) << "\n"
            //     << pos(1) << "\n"
            //     << pos(2) << "\n"
            //     << "vel: \n"
            //     << vel(0) << "\n"
            //     << vel(1) << "\n"
            //     << vel(2) << "\n"
            //     << "accel: \n"
            //     << accel(0) << "\n"
            //     << accel(1) << "\n"
            //     << accel(2) << "\n"
            //     << "jerk: \n"
            //     << jerk(0) << "\n"
            //     << jerk(1) << "\n"
            //     << jerk(2) << "\n"
            // << std::endl;

            // build up QuadGoal
            // acl_fsw::QuadGoal quad_goal;
            // quad_goal.cut_power = false;
            // quad_goal.xy_mode = acl_fsw::QuadGoal::MODE_ACCEL;
            // quad_goal.z_mode = acl_fsw::QuadGoal::MODE_POS;

            // Vector3 pos = TransformOrthoBodyToWorld(best_motion.getPosition(0.5));
            // Vector3 vel = RotateOrthoBodyToWorld(best_motion.getVelocity(0.5));
            // Vector3 accel = RotateOrthoBodyToWorld(best_motion.getAcceleration());
            // Vector3 jerk = RotateOrthoBodyToWorld(best_motion.getJerk());
                
            // quad_goal.jerk.x = jerk(0);
            // quad_goal.jerk.y = jerk(1);
            // quad_goal.jerk.z = jerk(2);
            // quad_goal.accel.x = accel(0);
            // quad_goal.accel.y = accel(1);
            // quad_goal.accel.z = accel(2);
            // //quad_goal.vel.x = vel(0);
            // //quad_goal.vel.y = vel(1);
            // //quad_goal.pos.x = pos(0);
            // //quad_goal.pos.y = pos(1);
            // if (use_3d_library) {
            //     quad_goal.pos.z = pos(2);
            //     quad_goal.vel.z = vel(2);
            // } else {
            //     double vel = 0;
            //     double accel = 0;
            //     double dolphin_altitude = DolphinStrokeDetermineAltitude(speed, vel, accel);
            //     carrot_world_frame(2) = dolphin_altitude; 
            //     quad_goal.pos.z = dolphin_altitude;
            //     quad_goal.vel.z = vel;
            //     quad_goal.accel.z = accel;
            // }

            // UpdateYaw();
            // quad_goal.yaw = -set_bearing_azimuth_degrees*M_PI/180.0;

            // if (stationary_yawing) {
            //     quad_goal.xy_mode = acl_fsw::QuadGoal::MODE_VEL;
            //     quad_goal.jerk.x = 0;
            //     quad_goal.jerk.y = 0;
            //     quad_goal.vel.x = 0;
            //     quad_goal.vel.y = 0;
            // }

            // quad_goal_pub.publish(quad_goal);
        }
    }

    bool motion_primitives_live = false;
    void OnCommand(const std_msgs::Int32& msg)  {
       if (msg.data == 1){
            ROS_INFO("GOT COMMAND ON");
            motion_primitives_live = true;
        }
        else{
            ROS_INFO("GOT COMMAND OFF");
            motion_primitives_live = false;
        }	
    }

    // void OnCommand(const fla_msgs::FlightCommand& msg)  {
    //    if (msg.command == fla_msgs::FlightCommand::CMD_GO){
    //         ROS_INFO("GOT GO COMMAND");
    //         motion_primitives_live = true;

    //         ROS_INFO("Starting");	
    //     }
    //     else{
    //         motion_primitives_live = false;
    //     }	
    // }

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
                if (use_acl && (speed_initial < 0.5)) {
                    bearing_azimuth_degrees = CalculateYawFromPosition(carrot_world_frame);
                    stationary_yawing = true;
                    return;
                }
                if (speed_initial < 0.5) {
                    bearing_azimuth_degrees = CalculateYawFromPosition(carrot_world_frame);
                    stationary_yawing = true;
                }

            }
        }
    }

    double CalculateYawFromPosition(Vector3 final_position) {
        return 180.0/M_PI*atan2(-(final_position(1) - pose_global_y), final_position(0) - pose_global_x);	
    }
    
    void UpdateMotionLibraryRollPitch(double roll, double pitch) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {
            motion_library_ptr->setRollPitch(roll, pitch);
        }
    }

    void PublishOrthoBodyTransform(double roll, double pitch) {
        static tf2_ros::TransformBroadcaster br;
          geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = body_frame_id;
        transformStamped.child_frame_id = "ortho_body";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q_ortho;
        q_ortho.setRPY(-roll, -pitch, 0);
        transformStamped.transform.rotation.x = q_ortho.x();
        transformStamped.transform.rotation.y = q_ortho.y();
        transformStamped.transform.rotation.z = q_ortho.z();
        transformStamped.transform.rotation.w = q_ortho.w();

        br.sendTransform(transformStamped);
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

    void UpdateAttitudeGeneratorRollPitch(double roll, double pitch) {
        attitude_generator.UpdateRollPitch(roll, pitch);
    }

    // void OnLidarlite(sensor_msgs::Range const& msg) {
    //     if (use_lidar_lite_z) {
    //         attitude_generator.setZ(msg.range);
    //     }
    // }

    void OnOdometry(const nav_msgs::Odometry& odom) {
        if (!got_camera_info) {
            // JON: added this for safety
            return;
        }
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        OnPose(pose_stamped);

        geometry_msgs::TwistStamped twist_stamped;
        twist_stamped.header = odom.header;
        twist_stamped.twist = odom.twist.twist;
        OnVelocity(twist_stamped);
    }

    ros::Time last_pose_update;
    void OnPose( geometry_msgs::PoseStamped const& pose ) {

        if ((ros::Time::now() - last_point_cloud_received).toSec() > 0.1) {
            // JON Added this update to always keep rotations up to date
            // DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
            // if (depth_image_collision_ptr != nullptr) {
            //     Matrix3 R = GetOrthoBodyToRDFRotationMatrix();
            //     Matrix3 R_set = GetBodyToRDFRotationMatrix();
            //     depth_image_collision_ptr->nanomap.SetBodyToRdf(R_set);
            //     depth_image_collision_ptr->UpdateRotationMatrix(R);
            //     depth_image_collision_ptr->UpdateBodyToRdf(R_set);
            // }
            ReactToSampledPointCloud();
        }
        motion_selector.UpdateCurrentAltitude(pose.pose.position.z);
        // ROS_INFO("GOT POSE");

        tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // if (!use_lidar_lite_z) {
        attitude_generator.setZ(pose.pose.position.z);
        // }
        last_pose_update = pose.header.stamp;
        UpdateMotionLibraryRollPitch(roll, pitch);
        UpdateAttitudeGeneratorRollPitch(roll, pitch);
        PublishOrthoBodyTransform(roll, pitch);
        UpdateCarrotOrthoBodyFrame();

        // double vel = 0.0;
        // double accel = 0.0;
        // double dolphin_altitude = DolphinStrokeDetermineAltitude(speed, vel, accel);
        // if (!use_3d_library) {
        //     carrot_world_frame(2) = dolphin_altitude; 
        //     attitude_generator.setZsetpoint(dolphin_altitude);
        // }

        ComputeBestAccelerationMotion();
        SetPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
        // PublishHealthStatus();

        Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
        NanoMapPose nm_pose(pos, quat, nm_time);

        Matrix4 transform = Eigen::Matrix4d::Identity();
        transform.block<3,3>(0,0) = nm_pose.quaternion.toRotationMatrix();
        transform.block<3,1>(0,3) = nm_pose.position;
        nanomap_visualizer.SetLastPose(transform);

        DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
        if (depth_image_collision_ptr != nullptr) {
            depth_image_collision_ptr->nanomap.AddPose(nm_pose);
        }
    }

    // void PublishHealthStatus() {

    // 	fla_msgs::ProcessStatus msg;
    // 	msg.id = 21; // 21 is motion_primitives status_id
    // 	msg.pid = getpid();

    // 	msg.status = fla_msgs::ProcessStatus::READY;
    // 	msg.arg = 0;

    // 	if (!got_camera_info && use_depth_image) {
    // 		msg.status = fla_msgs::ProcessStatus::ALARM;
    // 		msg.arg = 1;  
    // 	}
        
    // 	status_pub.publish(msg);
    // }

    void SetPose(double x, double y, double z, double yaw) {
        pose_global_x = x;
        pose_global_y = y;
        pose_global_z = z;
        pose_global_yaw = yaw;
    }

    Vector3 transformOrthoBodyIntoRDFFrame(Vector3 const& ortho_body_vector) {
        geometry_msgs::TransformStamped tf;
        if (!got_camera_info) {
            return Vector3(0,0,0);
        }
        try {
             tf = tf_buffer_.lookupTransform(depth_sensor_frame_id, "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
           } catch (tf2::TransformException &ex) {
              ROS_ERROR("ID 4 %s", ex.what());
          return Vector3(0,0,0);
        }
        geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_vector, "ortho_body");
        geometry_msgs::PoseStamped pose_vector_rdf_frame = PoseFromVector3(Vector3(0,0,0), depth_sensor_frame_id);
        tf2::doTransform(pose_ortho_body_vector, pose_vector_rdf_frame, tf);
        return VectorFromPose(pose_vector_rdf_frame);
    }

    Matrix3 GetOrthoBodyToRDFRotationMatrix() {
        geometry_msgs::TransformStamped tf;
        if (!got_camera_info) {
            return Matrix3();
        }
        try {
             tf = tf_buffer_.lookupTransform(depth_sensor_frame_id, "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
           } catch (tf2::TransformException &ex) {
              ROS_ERROR("ID 5 %s", ex.what());
          return Matrix3();
        }
        Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        Matrix3 R = quat.toRotationMatrix();
        return R;
    }

    Matrix3 GetBodyToRDFRotationMatrix() {
        geometry_msgs::TransformStamped tf;
        if (!got_camera_info) {
            std::cout << "" << std::endl;
            ROS_WARN("have not got camera info yet. Returning default_body_to_rdf");
            // JON: added this for safety
            return default_body_to_rdf;
        }
        try {
             tf = tf_buffer_.lookupTransform(depth_sensor_frame_id, body_frame_id, 
                                    ros::Time(0), ros::Duration(1/30.0));
           } catch (tf2::TransformException &ex) {
              ROS_ERROR("ID 5 %s", ex.what());
          return default_body_to_rdf;
        }
        Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        Matrix3 R = quat.toRotationMatrix();
        if (!R.isApprox(default_body_to_rdf)) {
            ROS_WARN("BodyToRDF is not approximately correct");
            std::cout << "R:" << std::endl;
            std::cout << R << std::endl;
            std::cout << "default_body_to_rdf" << std::endl;
            std::cout << default_body_to_rdf << std::endl;
        }

        return R;
    }

    Vector3 TransformWorldToOrthoBody(Vector3 const& world_frame) {
        geometry_msgs::TransformStamped tf;
        try {
          tf = tf_buffer_.lookupTransform("ortho_body", "world", 
                                        ros::Time(0), ros::Duration(1/30.0));
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("ID 6 %s", ex.what());
          return Vector3(1,1,1);
        }

        Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        Matrix3 R = quat.toRotationMatrix();
        return R*world_frame;
    }

    Vector3 RotateOrthoBodyToWorld(Vector3 const& ortho_body_frame) {
        geometry_msgs::TransformStamped tf;
        try {
          tf = tf_buffer_.lookupTransform("world", "ortho_body", 
                                        ros::Time(0), ros::Duration(1/30.0));
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("ID 6 %s", ex.what());
          return Vector3(1,1,1);
        }

        Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        Matrix3 R = quat.toRotationMatrix();
        return R*ortho_body_frame;
    }

    void UpdateMotionLibraryVelocity(Vector3 const& velocity_ortho_body_frame) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
        if (motion_library_ptr != nullptr) {
            motion_library_ptr->setInitialVelocity(velocity_ortho_body_frame);
        }
    }

    void OnVelocity( geometry_msgs::TwistStamped const& twist) {
        //ROS_INFO("GOT VELOCITY");
        attitude_generator.setZvelocity(twist.twist.linear.z);
        Vector3 velocity_world_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
        Vector3 velocity_ortho_body_frame = TransformWorldToOrthoBody(velocity_world_frame);
        if (!use_3d_library || !use_acl) {
            velocity_ortho_body_frame(2) = 0.0;  // WARNING for 2D only
        }
        UpdateMotionLibraryVelocity(velocity_ortho_body_frame);
        speed = velocity_ortho_body_frame.norm();
        MonitorProgress();
        //UpdateTimeHorizon(speed);
        UpdateMaxAcceleration(speed);
    }

    ros::Time time_last_made_progress;
    bool progress_initialized = false; 
    bool making_progress;
    double progress_timer = 3.0;
    double velocity_progress_threshold = 0.5;
    double yaw_progress_threshold = 30.0;
    void MonitorProgress() {
        if (!progress_initialized) {
            time_last_made_progress = ros::Time::now();
            progress_initialized = true;
        }
        if (speed > velocity_progress_threshold) {
            time_last_made_progress = ros::Time::now();
        }
        double actual_bearing_azimuth_degrees = -pose_global_yaw * 180.0/M_PI;
        double actual_bearing_error = bearing_azimuth_degrees - actual_bearing_azimuth_degrees;
        if (abs(actual_bearing_error) > yaw_progress_threshold) {
            time_last_made_progress = ros::Time::now();
        }
        if ((ros::Time::now() - time_last_made_progress).toSec() > progress_timer) {
            //std::cout << "NOT MAKING PROGRESS" << std::endl;
        }
    }

    void UpdateMaxAcceleration(double speed) {
        MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
            if (motion_library_ptr != nullptr) {
                motion_library_ptr->UpdateMaxAcceleration(speed);
            }
    }

    void UpdateTimeHorizon(double speed) { 
        if (speed < 10.0) {
            final_time = 1.0;
        }
        else { 
            final_time = 10.0 / speed;
        }
        if (final_time < 1.0) { final_time = 1.0;}
        motion_visualizer.UpdateTimeHorizon(final_time);
        motion_selector.UpdateTimeHorizon(final_time);
    }
    
    Vector3 TransformOrthoBodyToWorld(Vector3 const& ortho_body_frame) {
        geometry_msgs::TransformStamped tf;
        try {
          tf = tf_buffer_.lookupTransform("world", "ortho_body",
                                        ros::Time(0), ros::Duration(1/30.0));
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("ID 7 %s", ex.what());
          return Vector3::Zero();
        }

        geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_frame, "ortho_body");
        geometry_msgs::PoseStamped pose_vector_world_frame = PoseFromVector3(Vector3(0,0,0), "world");
        tf2::doTransform(pose_ortho_body_vector, pose_vector_world_frame, tf);

        return VectorFromPose(pose_vector_world_frame);
    }

    // void ProjectOrthoBodyLaserPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) {
    //     pcl::PointCloud<pcl::PointXYZ>::iterator point_cloud_iterator_begin = cloud_ptr->begin();
    //     pcl::PointCloud<pcl::PointXYZ>::iterator point_cloud_iterator_end = cloud_ptr->end();

    //     for (pcl::PointCloud<pcl::PointXYZ>::iterator point = point_cloud_iterator_begin; point != point_cloud_iterator_end; point++) {
    //         if (point->z > laser_z_below_project_up) {
    //             point->z = 0.0;
    //         }
    //     }
    // }


    // void OnScan(sensor_msgs::PointCloud2ConstPtr const& laser_point_cloud_msg) {
    //     DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
    //     if (depth_image_collision_ptr != nullptr) {
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr ortho_body_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //         TransformToOrthoBodyPointCloud("laser", laser_point_cloud_msg, ortho_body_cloud);
    //         if (!use_3d_library) {
    //              ProjectOrthoBodyLaserPointCloud(ortho_body_cloud);
    //         }
    //         depth_image_collision_ptr->UpdateLaserPointCloudPtr(ortho_body_cloud);
    //     }
    // }

    void OnSmoothedPoses(nav_msgs::Path path) {
        DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
        if (depth_image_collision_ptr != nullptr) {
            std::vector<NanoMapPose> smoothed_path_vector;

            size_t path_size = path.poses.size();
            for (size_t i = 0; i < path_size; i++) {
                geometry_msgs::PoseStamped pose = path.poses.at(i);
                Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
                Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
                NanoMapPose nm_pose(pos, quat, nm_time);
                smoothed_path_vector.push_back(nm_pose);
            }

            depth_image_collision_ptr->nanomap.AddPoseUpdates(smoothed_path_vector);
        }	
    }


    // JON: dolphin is only enabled if !use_3d_library

    // Hack function to add motion up/down in order to help state estimator
    // dolphin_altitude(t) = A * cos(t * 2pi/T) + flight_altitude
    // A = amplitude
    // T = period
    // double A_dolphin = 0.5;
    // double T_dolphin = 3.0;
    // double dolphin_acceleration_threshold = 2.0;
    // double cooldown_duration = 3.0;

    // double time_of_last_dolphin_query = 0;
    // bool dolphin_initialized = false;
    // double time_of_start_cooldown = 0;

    // double dolphin_offset = 0.0;

    // size_t cooldown_hit_counter = 0;
    // size_t cooldown_hit_threshold = 20;
    // bool in_cooldown = false;

    // double DolphinStrokeDetermineAltitude(double speed, double &vel, double &accel) {
    //     if (!dolphin_initialized) {
    //         dolphin_initialized = true;
    //         time_of_start_dolphin_stroke = ros::Time::now().toSec();
    //         time_of_start_cooldown = ros::Time::now().toSec();
    //         return flight_altitude;
    //     }

    //     double time_now = ros::Time::now().toSec();

    //     if (desired_acceleration.norm() > dolphin_acceleration_threshold) {
    //         cooldown_hit_counter++;
    //         if (cooldown_hit_counter > cooldown_hit_threshold) {
    //             cooldown_hit_counter = 0;
    //             time_of_start_cooldown = time_now;
    //             in_cooldown = true;	
    //         }
    //     }
    //     else {
    //         if (cooldown_hit_counter <= 1) {
    //             cooldown_hit_counter = 0;
    //         }
    //         else {
    //             cooldown_hit_counter--;
    //         }
    //     }
        
    //     if ( (time_now - time_of_start_cooldown) > cooldown_duration) {
    //         if (in_cooldown) {
    //             in_cooldown = false;
    //             time_of_start_dolphin_stroke = time_now;
    //         }
    //         double time_since_start_dolphin = time_now - time_of_start_dolphin_stroke;
    //         dolphin_offset = A_dolphin * sin(time_since_start_dolphin * 2 * M_PI / T_dolphin);
    //         vel = A_dolphin * cos(time_since_start_dolphin * 2 * M_PI/T_dolphin) * 2 * M_PI / T_dolphin;
    //         accel = A_dolphin * -sin(time_since_start_dolphin * 2 * M_PI/T_dolphin) * 4 * M_PI * M_PI / (T_dolphin * T_dolphin);

    //     }
    //     else {
    //         if (dolphin_offset > 0) {dolphin_offset = dolphin_offset - 0.01;}
    //         else {dolphin_offset = dolphin_offset + 0.01;}
    //     }

    //     double ans = dolphin_offset + flight_altitude;

    //     geometry_msgs::PoseStamped pose;
    //     pose.pose.position.z = ans;
    //     pose.header.stamp = ros::Time::now();
    //     pose.header.frame_id = "world";
    //     attitude_setpoint_visualization_pub.publish(pose);

    //     return ans;
    // }


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
        carrot_pub.publish( marker );
    }

    void TransformToOrthoBodyPointCloud(std::string const& source_frame, const sensor_msgs::PointCloud2ConstPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out){
          sensor_msgs::PointCloud2 msg_out;
          geometry_msgs::TransformStamped tf;
        try {
             tf = tf_buffer_.lookupTransform("ortho_body", source_frame,
                        ros::Time(0), ros::Duration(1/30.0));
               } catch (tf2::TransformException &ex) {
                  ROS_ERROR("8 %s", ex.what());
          return;
        }
          Eigen::Quaternionf quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        Eigen::Matrix3f R = quat.toRotationMatrix();
        Eigen::Vector4f T = Eigen::Vector4f(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z, 1.0); 

         Eigen::Matrix4f transform_eigen; // Your Transformation Matrix
        transform_eigen.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        transform_eigen.block<3,3>(0,0) = R;
        transform_eigen.col(3) = T;

          pcl_ros::transformPointCloud(transform_eigen, *msg, msg_out);

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(msg_out, cloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(cloud2,*cloud);

        cloud_out = cloud;
    }

    ros::Time last_point_cloud_received;
    void OnDepthImage(const sensor_msgs::PointCloud2& point_cloud_msg) {
        if (!got_camera_info) {
            // JON: added this for safety
            return;
        }
        if (UseDepthImage()) {
            last_point_cloud_received = ros::Time::now();
            DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();
            if (depth_image_collision_ptr != nullptr) {
                Matrix3 R = GetOrthoBodyToRDFRotationMatrix();
                Matrix3 R_set = GetBodyToRDFRotationMatrix();
                depth_image_collision_ptr->nanomap.SetBodyToRdf(R_set);
                depth_image_collision_ptr->UpdateRotationMatrix(R);
                depth_image_collision_ptr->UpdateBodyToRdf(R_set);
                if(use_depth_image) {
                    pcl::PCLPointCloud2 cloud2_rdf;
                    pcl_conversions::toPCL(point_cloud_msg, cloud2_rdf);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rdf(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::fromPCLPointCloud2(cloud2_rdf,*cloud_rdf);
                    NanoMapTime nm_time(point_cloud_msg.header.stamp.sec, point_cloud_msg.header.stamp.nsec);
                    // ROS_INFO("AddPointCloud");
                    depth_image_collision_ptr->nanomap.AddPointCloud(cloud_rdf, nm_time, point_cloud_msg.header.seq);
                }
            }
            ReactToSampledPointCloud();
        }
    }

    void UpdateYaw() {
        // // Limit size of bearing errors
        double bearing_error_cap = 30;
        double actual_bearing_azimuth_degrees = -pose_global_yaw * 180.0/M_PI;
        double actual_bearing_error = bearing_azimuth_degrees - actual_bearing_azimuth_degrees;
        while(actual_bearing_error > 180) { 
            actual_bearing_error -= 360;
        }
        while(actual_bearing_error < -180) { 
            actual_bearing_error += 360;
        }
        if (actual_bearing_error > bearing_error_cap) {
            bearing_azimuth_degrees = actual_bearing_azimuth_degrees + bearing_error_cap;
        }
        if (actual_bearing_error < -bearing_error_cap) {
            bearing_azimuth_degrees = actual_bearing_azimuth_degrees - bearing_error_cap;
        }

        double bearing_error = bearing_azimuth_degrees - set_bearing_azimuth_degrees;

        while(bearing_error > 180) { 
            bearing_error -= 360;
        }
        while(bearing_error < -180) { 
            bearing_error += 360;
        }

        if (abs(bearing_error) < 1.0) {
            set_bearing_azimuth_degrees = bearing_azimuth_degrees;
        }

        else if (bearing_error < 0)  {
            set_bearing_azimuth_degrees -= 1.0;
        }
        else {
            set_bearing_azimuth_degrees += 1.0;
        }
        if (set_bearing_azimuth_degrees > 180.0) {
            set_bearing_azimuth_degrees -= 360.0;
        }
        if (set_bearing_azimuth_degrees < -180.0) {
            set_bearing_azimuth_degrees += 360.0;
        }
    }

    void PublishAttitudeSetpoint(Vector3 const& roll_pitch_thrust) { 

        using namespace Eigen;

        mavros_msgs::AttitudeTarget setpoint_msg;
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE 
            | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE
            | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE
            ;

        UpdateYaw();

        Matrix3f m;
        m =AngleAxisf(-set_bearing_azimuth_degrees*M_PI/180.0, Vector3f::UnitZ())
        * AngleAxisf(roll_pitch_thrust(1), Vector3f::UnitY())
        * AngleAxisf(-roll_pitch_thrust(0), Vector3f::UnitX());
        
        Quaternionf q(m);

        setpoint_msg.orientation.w = q.w();
        setpoint_msg.orientation.x = q.x();
        setpoint_msg.orientation.y = q.y();
        setpoint_msg.orientation.z = q.z();

        setpoint_msg.thrust = roll_pitch_thrust(2);

        attitude_thrust_pub.publish(setpoint_msg);
    }


    ros::Subscriber camera_info_sub;
    ros::Subscriber odom_sub;
    // ros::Subscriber pose_sub;
    // ros::Subscriber velocity_sub;
    // ros::Subscriber height_above_ground_sub;
    ros::Subscriber depth_image_sub;
    // ros::Subscriber global_goal_sub;
    ros::Subscriber local_goal_sub;
    // ros::Subscriber laser_scan_sub;
    // ros::Subscriber max_speed_sub;
    ros::Subscriber smoothed_pose_sub;
    ros::Subscriber command_sub;

    ros::Publisher carrot_pub;
    ros::Publisher gaussian_pub;
    ros::Publisher attitude_thrust_pub;
    ros::Publisher attitude_setpoint_visualization_pub;
    // ros::Publisher status_pub;
    // ros::Publisher quad_goal_pub;
    ros::Publisher waypoints_pub;

    std::string depth_sensor_frame_id = "depth_sensor";
    std::string body_frame_id = "body";

    std::vector<ros::Publisher> action_paths_pubs;
    tf::TransformListener listener;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2_ros::Buffer tf_buffer_;

    double start_time = 0.0;
    double final_time = 1.5;

    double bearing_azimuth_degrees = 0.0;
    double set_bearing_azimuth_degrees = 0.0;

    Eigen::Vector4d pose_x_y_z_yaw;

    Matrix3 default_body_to_rdf; // from NWU to right-down-front (depth frame)

    Vector3 carrot_world_frame;
    Vector3 carrot_ortho_body_frame = Vector3(0,0,0);

    size_t best_traj_index = 0;
    Vector3 desired_acceleration = Vector3(0,0,0);

    MotionSelector motion_selector;
    AttitudeGenerator attitude_generator;
    NanoMapVisualizer nanomap_visualizer;

    double pose_global_x = 0;
    double pose_global_y = 0;
    double pose_global_z = 0;
    double pose_global_yaw = 0;

    bool yaw_on = false;
    double soft_top_speed_max = 0.0;
    bool use_depth_image = true;
    // bool use_lidar_lite_z = false;
    bool use_3d_library = false;
    bool use_acl = true;
    double flight_altitude = 1.0;

    bool executing_e_stop = false;
    double begin_e_stop_time = 0.0;
    double e_stop_time_needed = 0.0;
    double max_e_stop_pitch_degrees = 60.0;

    // double laser_z_below_project_up = -0.5;
    double time_of_start_dolphin_stroke = 0.0;
    double speed = 0.0;

    enum StatusArg {
            NOMINAL = 0
    };

    ros::NodeHandle nh;

public:
    MotionVisualizer motion_visualizer;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


int main(int argc, char* argv[]) {
    std::cout << "Initializing nanomap_navigation_node" << std::endl;

    ros::init(argc, argv, "NanoMapNavigationNode");

    NanoMapNavigationNode motion_selector_node;

    ros::Rate spin_rate(100);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();

    size_t counter = 0;

    while (ros::ok()) {
        //t1 = std::chrono::high_resolution_clock::now();
        //motion_selector_node.ReactToSampledPointCloud();
        //t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "ReactToSampledPointCloud took "
  //     		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
  //     		<< " microseconds\n";
        motion_selector_node.PublishCurrentAttitudeSetpoint();

        counter++;
        if (counter > 3) {
            counter = 0;
            motion_selector_node.drawAll();
            if (!motion_selector_node.UseDepthImage()) {
                motion_selector_node.ReactToSampledPointCloud();
            }
        }
          
        ros::spinOnce();
        spin_rate.sleep();
    }
}
