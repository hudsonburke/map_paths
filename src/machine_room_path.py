#!/usr/bin/env python3

import math
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class MachineRoomPath:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.points = [
            [1.14, 0, 90],
            [1.14, 1.2, 180],
            [0.8, 1.2, 270],
            #[0.44, 0.44, 315],
            [0, 0, 0],
            [1.14, 0, 90],
            [1.14, 1.2, 180],
            [0.8, 1.2, 270],
            #[0.44, 0.44, 315],
            [0, 0, 0]
        ]
        self.num_goals = len(self.points)
        self.goal_cnt = 0
        if len(self.points) > 0:
            self.navigate(*self.points[self.goal_cnt], "map")
        else:
            rospy.loginfo("No goals given.")
            rospy.signal_shutdown("No goals given.")
            return

    def navigate(self, x, y, yaw=0.0, frame="map"):  # yaw in degrees    
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
    
        (self.goal.target_pose.pose.orientation.x, self.goal.target_pose.pose.orientation.y,
        self.goal.target_pose.pose.orientation.z, self.goal.target_pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw*math.pi/180)
        # default: no rotation

        self.client.send_goal(self.goal, self.done_cb)
        if self.goal_cnt == 0:
             rospy.spin()
        rate = rospy.Rate(5)
        rate.sleep()

        
        

        # # Wait for server to finish
        # wait = self.client.wait_for_result()
        # # If the result doesn't arrive, assume the Server is not available
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     # Result of executing the action
        #     result = self.client.get_result()
        #     if result:
        #         rospy.loginfo("Goal execution done!")
        #         return True

    def done_cb(self, status, result):
        self.goal_cnt += 1
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< self.num_goals:
                rospy.loginfo("Starting toward pose "+str(self.goal_cnt+1))
                self.navigate(*self.points[self.goal_cnt], "map")
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
    
        


if __name__ == "__main__":
    try:
        rospy.init_node('machine_room_path')
        rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.25)
        rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance', 45*math.pi/180)
        # rospy.set_param('/move_base/TrajectoryPlannerROS/max_vel_theta', 0.5)
        # rospy.set_param('/move_base/TrajectoryPlannerROS/min_vel_theta', -0.5)
        # rospy.set_param('/move_base/TrajectoryPlannerROS/acc_lim_theta', 0.15)
        # rospy.set_param('/move_base/DWAPlannerROS/max_vel_theta', 0.5)
        # rospy.set_param('/move_base/DWAPlannerROS/min_vel_theta', -0.5)
        # rospy.set_param('/move_base/DWAPlannerROS/acc_lim_theta', 0.15)

        # while not rospy.is_shutdown():
        MachineRoomPath()
        # if not rospy.is_shutdown():
        #     for point in points:
        #         path.navigate(point[0], point[1], point[2], "odom")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Finished")
