#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_from_matrix
from scipy.spatial.transform import Rotation

# TODO bring this out to flight plotter

class TrajectoryHistoryNode:
    def __init__(self, traj_draw_rate=10, pose_draw_rate=1, history_length=10, redraw_history=False):
        '''
        history_length: how much of trajectory to over
        redraw_history: overwrites trajectory history visualizations
        '''
        self.traj_draw_rate = traj_draw_rate
        self.pose_draw_rate = pose_draw_rate 
        self.points_history = []
        self.history_length = history_length
        self.redraw_history = redraw_history  

        rospy.init_node('trajectory_history_node')

        # Publishers
        self.traj_pub = rospy.Publisher('trajectory_history', Marker, queue_size=1)
        self.pose_pub = rospy.Publisher('trajectory_poses', MarkerArray, queue_size=1)

        # Subscribers
        self.pose_sub = rospy.Subscriber('~pose_topic', PoseWithCovarianceStamped, self.pose_callback)

        self.id_count = 0
        self.last_traj_draw_time = 0.0
        self.last_pose_draw_time = 0.0

    @classmethod
    def draw_linestrip(self, points, marker_pub, stamp, namespace='line_strip', id=0, frame_id='world', line_width=0.02, rgba=(1.0, 0.0, 0.0, 1.0)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0 # default orientation
        marker.scale.x = line_width
        r, g, b, a = rgba
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.points = points

        # add points to linestrip
        marker_pub.publish(marker)

    @classmethod
    def draw_frame(self, pose, marker_pub, stamp=None, scale=0.5, namespace='', id=0, frame_id='world', line_width=0.01):

        if (stamp is None):
            stamp = rospy.Time.now()

        marker_array = MarkerArray()
        marker_array.markers = []

        # Using scipy.spatial.transform.Rotation for to compute axes directions
        r_orientation = Rotation.from_quat((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
        offset_body = np.array([0, 0, scale/2])
        position_body = np.array([pose.position.x, pose.position.y, pose.position.z])
 
        # Create coordinate frame markers from 3 cylinders
        z_axis = Marker()
        z_axis.header.frame_id = frame_id
        z_axis.header.stamp = stamp
        z_axis.ns = namespace
        z_axis.type = Marker.CYLINDER
        z_axis.action = Marker.ADD
        z_axis.id = id
        corrected_position = position_body + r_orientation.apply(offset_body)
        z_axis.pose.position = Point(*corrected_position)
        z_axis.pose.orientation = Quaternion(*r_orientation.as_quat())
        z_axis.scale = Vector3(line_width, line_width, scale)
        z_axis.color = ColorRGBA(0,0,1,1) # blue
        marker_array.markers.append(z_axis)

        # rotate about y axis to get x axis
        r_z_x = Rotation.from_rotvec(0.5*np.pi*np.array([0, 1, 0]))
        r_x = r_orientation * r_z_x 
        x_axis = Marker()
        x_axis.header.frame_id = frame_id
        x_axis.header.stamp = stamp
        x_axis.ns = namespace
        x_axis.type = Marker.CYLINDER
        x_axis.action = Marker.ADD
        x_axis.id = id + 1
        corrected_position = position_body + r_x.apply(offset_body)
        x_axis.pose.position = Point(*corrected_position)
        x_axis.pose.orientation = Quaternion(*r_x.as_quat())
        x_axis.scale = Vector3(line_width, line_width, scale)
        x_axis.color = ColorRGBA(1,0,0,1) # red
        marker_array.markers.append(x_axis)

        # rotate about x axis to get y axis
        r_z_y = Rotation.from_rotvec(-0.5*np.pi*np.array([1,0,0]))
        r_y = r_orientation * r_z_y
        y_axis = Marker()
        y_axis.header.frame_id = frame_id
        y_axis.header.stamp = stamp
        y_axis.ns = namespace
        y_axis.type = Marker.CYLINDER
        y_axis.action = Marker.ADD
        y_axis.id = id + 2
        corrected_position = position_body + r_y.apply(offset_body)
        y_axis.pose.position = Point(*corrected_position)
        y_axis.pose.orientation = Quaternion(*r_y.as_quat())
        y_axis.scale = Vector3(line_width, line_width, scale)
        y_axis.color = ColorRGBA(0,1,0,1) # green
        marker_array.markers.append(y_axis)

        marker_pub.publish(marker_array)

    def pose_callback(self, pose_with_covariance_msg):
        if (rospy.Time.now().to_sec() - self.last_traj_draw_time > 1/self.traj_draw_rate):
            self.last_draw_time = rospy.Time.now().to_sec()

            pose = pose_with_covariance_msg.pose.pose
            stamp = pose_with_covariance_msg.header.stamp
            self.points_history.append(pose.position)
            if (len(self.points_history) > self.history_length):
                self.points_history.pop(0)

            # draw the trajectory history as orange linestrip
            color = (1.0, 0.5, 0.0, 1.0) 
            self.draw_linestrip(self.points_history, self.traj_pub, stamp, id=self.id_count, rgba=color)

        if (rospy.Time.now().to_sec() - self.last_pose_draw_time > 1/self.pose_draw_rate):
            self.last_pose_draw_time = rospy.Time.now().to_sec()
        
            self.draw_frame(pose, self.pose_pub, stamp, scale=0.1, id=self.id_count)
        
        if (not self.redraw_history): 
            # incrementing count prevents marker id from being overwritten
            self.id_count += 3 


if __name__ == '__main__':
    try:
        node = TrajectoryHistoryNode(traj_draw_rate=10, pose_draw_rate=4)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass