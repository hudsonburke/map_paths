#!/usr/bin/env python3

import math
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def navigate(x, y, yaw=0.0, frame="map"):  # yaw in degrees
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    (goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
     goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw*math.pi/180)
    # default: no rotation

    client.send_goal(goal)

    # Wait for server to finish
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        result = client.get_result()
        if result:
            rospy.loginfo("Goal execution done!")
            return True


if __name__ == "__main__":
    try:
        rospy.init_node('machine_room_path', anonymous=True)
        rospy.set_param('xy_goal_tolerance', 0.25)
        rospy.set_param('theta_goal_tolerance', 10)
        points = [
            [1.14, 0, -90],
            [1.14, 1.2, -180],
            [0.8, 1.2, -270],
            [0.44, 0.44, -315],
            [0, 0, 0],
            [1.14, 0, -90],
            [1.14, 1.2, -180],
            [0.8, 1.2, -270],
            [0.44, 0.44, -315],
            [0, 0, 0]
        ]
        if not rospy.is_shutdown():
            for point in points:
                navigate(point[0], point[1], point[2], "odom")
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
