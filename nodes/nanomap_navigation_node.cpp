#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nanomap_navigation/motion_selector.h>
#include <nanomap_navigation/motion_visualizer.h>

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
    pose_sub = nh.subscribe("/rocky0704/pose", 100, &NanoMapNavigationNode::OnPose, this);
    velocity_sub = nh.subscribe("/rocky0704/twist", 100, &NanoMapNavigationNode::OnVelocity, this);

    local_goal_sub = nh.subscribe("local_goal_topic", 1, &MotionSelectorNode::OnLocalGoal, this);
    // Publishers
    // TODO publish visualizations
    }

    void drawAll() {
        motion_visualizer.drawAll();
    }

private:

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
        // SetPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
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

    ros::Subscriber pose_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber local_goal_sub;
  
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    // To initialize library
    bool use_3d_library = true;
    double final_time = 1.5;
    double flight_altitude = 1.0;
    double soft_top_speed_max = 0.0;

    size_t best_traj_index = 0;

    MotionSelector motion_selector;
    MotionVisualizer motion_visualizer;

    double speed = 0.0;

    ros::NodeHandle nh;

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