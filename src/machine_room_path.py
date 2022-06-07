#!/usr/bin/env python3

import math
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def navigate(x, y, yaw=0.0, frame="map"):  # yaw in degrees
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    (goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw*math.pi/180)
    # default: no rotation
    client.send_goal(goal, done_cb)


def done_cb(status, result):
    # https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/
    goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
    if status == 2:
        rospy.loginfo("Goal pose "+str(goal_cnt) +
                      " received a cancel request after it started executing, completed execution!")

    if status == 3:
        rospy.loginfo("Goal pose "+str(goal_cnt)+" reached")
        if goal_cnt < num_goals:
            rospy.loginfo("Starting toward pose "+str(goal_cnt+1))
            navigate(*points[goal_cnt], "map")
        else:
            rospy.loginfo("Final goal pose reached!")
            rospy.signal_shutdown("Final goal pose reached!")
            return

    if status == 4:
        rospy.loginfo("Goal pose "+str(goal_cnt) +
                      " was aborted by the Action Server")
        rospy.signal_shutdown(
            "Goal pose "+str(goal_cnt)+" aborted, shutting down!")
        return

    if status == 5:
        rospy.loginfo("Goal pose "+str(goal_cnt) +
                      " has been rejected by the Action Server")
        rospy.signal_shutdown(
            "Goal pose "+str(goal_cnt)+" rejected, shutting down!")
        return

    if status == 8:
        rospy.loginfo("Goal pose "+str(goal_cnt) +
                      " received a cancel request before it started executing, successfully cancelled!")


if __name__ == "__main__":
    try:
        rospy.init_node('machine_room_path')
        # http://wiki.ros.org/dwa_local_planner
        # https://kaiyuzheng.me/documents/navguide.pdf
        rospy.set_param('/move_base/DWAPlannerROS/', {
            "xy_goal_tolerance": 0.25,
            "yaw_goal_tolerance": 45*math.pi/180,
        })
        # http://wiki.ros.org/base_local_planner
        rospy.set_param('/move_base/TrajectoryPlannerROS/', {
            "xy_goal_tolerance": 0.25,
            "yaw_goal_tolerance": 45*math.pi/180,
        })

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        wait = client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        points = [
            [1.14, 0, 90],
            [1.14, 1.2, 180],
            [0.8, 1.2, 270],
            # [0.44, 0.44, 315],
            [0, 0, 0],
            [1.14, 0, 90],
            [1.14, 1.2, 180],
            [0.8, 1.2, 270],
            # [0.44, 0.44, 315],
            [0, 0, 0]
        ]

        num_goals = len(points)
        goal_cnt = 0
        rate = rospy.Rate(5)
        if num_goals > 0:
            while not rospy.is_shutdown():
                navigate(*points[goal_cnt], "map")
                rate.sleep()
        else:
            rospy.loginfo("No goals given.")
            rospy.signal_shutdown("No goals given.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Finished")
