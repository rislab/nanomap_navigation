#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class GoalNode:
    def __init__(self, goals, goal_radius=0.5):
        self.goals = goals
        self.goal_idx = 0
        self.goal_radius = goal_radius 

        rospy.init_node('goal_node')

        # Publishers
        self.goal_pub = rospy.Publisher('local_goal', PoseStamped, queue_size=0)

        # Subscribers
        self.pose_sub = rospy.Subscriber('~pose_topic', PoseWithCovarianceStamped, self.pose_callback)


    def pose_callback(self, pose_with_covariance_msg):
        pose = pose_with_covariance_msg.pose.pose

        cur_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        current_goal = self.goals[self.goal_idx]
        distance_to_goal = np.linalg.norm(cur_position - current_goal)
        if (distance_to_goal < self.goal_radius):
            print("Arrived at goal. Switching to next goal")
            # or just stop at last goal
            self.goal_idx = (self.goal_idx + 1) % len(self.goals) 

    def run(self):
        rate = rospy.Rate(10) # Hz
        seq = 1
        while not rospy.is_shutdown():
            goal = PoseStamped()

            goal.header.seq = seq
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "world"

            goal.pose.position.x = self.goals[self.goal_idx][0]
            goal.pose.position.y = self.goals[self.goal_idx][1]
            goal.pose.position.z = self.goals[self.goal_idx][2]

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)

            rate.sleep()
            seq += 1

if __name__ == '__main__':
    try:
        goals = [
            np.array([0, -18, 2]),
            np.array([-20, -18, 2]),
            np.array([0, -18, 2]),
            np.array([0, 0, 2]),
        ]
        node = GoalNode(goals, goal_radius=2)
        node.run()
    except rospy.ROSInterruptException:
        pass