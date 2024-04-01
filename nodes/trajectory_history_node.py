#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

# TODO bring this out to flight plotter

class TrajectoryHistoryNode:
    def __init__(self, history_length=None):
        self.points_history = []
        self.history_length = history_length

        rospy.init_node('trajectory_history_node')

        # Publishers
        self.traj_pub = rospy.Publisher('trajectory_history', Marker, queue_size=1)

        # Subscribers
        self.pose_sub = rospy.Subscriber('~pose_topic', PoseWithCovarianceStamped, self.pose_callback)

        self.id_count = 0

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


    def pose_callback(self, pose_with_covariance_msg):
        pose = pose_with_covariance_msg.pose.pose
        stamp = pose_with_covariance_msg.header.stamp
        self.points_history.append(pose.position)
        if (self.history_length is not None and len(self.points_history) > self.history_length):
            self.points_history.pop(0)

        # draw the trajectory history as orange linestrip
        color = (1.0, 0.5, 0.0, 1.0) 
        self.draw_linestrip(self.points_history, self.traj_pub, stamp, id=self.id_count, rgba=color)

        
        if (self.history_length is None): 
            # incrementing count prevents marker id from being overwritten
            self.id_count += 1 

if __name__ == '__main__':
    try:
        node = TrajectoryHistoryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass