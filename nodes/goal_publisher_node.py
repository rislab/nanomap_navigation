#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def goal_publisher(goals, goal_radius=1):
    pub = rospy.Publisher('local_goal', PoseStamped, queue_size=0)
    rospy.init_node('goal_publisher')
    rate = rospy.Rate(10) # Hz
    seq = 1
    while not rospy.is_shutdown():
        goal = PoseStamped()

        goal.header.seq = seq
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"

        # goal.pose.position.x = 0.0
        # goal.pose.position.y = -15.0
        # goal.pose.position.z = 2.0
        goal.pose.position.x = 10.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 2.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        pub.publish(goal)
        print(f"Publishing goal {seq}")
        rate.sleep()

        seq += 1

if __name__ == '__main__':
    try:
        goals = [
            (0, -20, 2),
        ]
        goal_publisher(goals, goal_radius=1)
    except rospy.ROSInterruptException:
        pass