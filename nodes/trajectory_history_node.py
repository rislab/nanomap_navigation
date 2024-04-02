#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_matrix

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
        self.pose_pub = rospy.Publisher('trajectory_poses', Marker, queue_size=1)

        # Subscribers
        self.pose_sub = rospy.Subscriber('~pose_topic', PoseWithCovarianceStamped, self.pose_callback)

        self.id_count = 0
        self.last_traj_draw_time = 0.0
        self.last_pose_draw_time = 0.0

    @classmethod
    def draw_linestrip(self, points, marker_pub, stamp, namespace='line_strip', id=0, frame_id='world', line_width=0.01, rgba=(1.0, 0.0, 0.0, 1.0)):
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
    def draw_frame(self, pose, marker_pub, stamp=None, scale=0.5, namespace='line_list', id=0, frame_id='world', line_width=0.05):
        marker = Marker()
        marker.header.frame_id = frame_id
        if (stamp is None):
            marker.header.stamp = rospy.Time.now()
        else:
            marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = Vector3(line_width, line_width, line_width)
        
        marker.points = [
            Point(0, 0, 0), Point(scale, 0, 0),
            Point(0, 0, 0), Point(0, scale, 0),
            Point(0, 0, 0), Point(0, 0, scale),
        ]

        # colors of coordinate axes (x: red, y: green, z: blue)
        marker.colors = [
            ColorRGBA(1, 0, 0, 1),
            ColorRGBA(1, 0, 0, 1),
            ColorRGBA(0, 1, 0, 1),
            ColorRGBA(0, 1, 0, 1),
            ColorRGBA(0, 0, 1, 1),
            ColorRGBA(0, 0, 1, 1),
        ]

        # add points to linestrip
        marker_pub.publish(marker)

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
        
            self.draw_frame(pose, self.pose_pub, stamp, scale=0.2, id=self.id_count)
        
        if (not self.redraw_history): 
            # incrementing count prevents marker id from being overwritten
            self.id_count += 1 


if __name__ == '__main__':
    try:
        node = TrajectoryHistoryNode(traj_draw_rate=10, pose_draw_rate=4)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass