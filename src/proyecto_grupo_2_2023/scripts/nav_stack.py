#!/usr/bin/env python3

import rospy as rp
import actionlib
import numpy as np

from tf.transformations import quaternion_from_euler

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Quaternion


class NavStack:
    def __init__(self):
        rp.init_node('nav_stack')

        self.move_base_client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def goal_done(self, status, result):
        rp.loginfo(
            f"DONE status: {self.move_base_client.get_goal_status_text()} " +
            f"({str(status)})")

    def set_goals(self, list_x, list_y, list_a):
        for x, y, a in zip(list_x, list_y, list_a):
            goal_pose = Pose()
            goal_pose.position.x = x
            goal_pose.position.y = y
            q = quaternion_from_euler(0, 0, a)
            goal_pose.orientation = Quaternion(*q)

            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = 'map'
            move_base_goal.target_pose.header.stamp = rp.Time.now()
            move_base_goal.target_pose.pose = goal_pose
            self.move_base_client.send_goal(
                move_base_goal, done_cb=self.goal_done)
            self.move_base_client.wait_for_result()


if __name__ == "__main__":
    nav_stack = NavStack()
    nav_stack.set_goals((1, 1, 3), (1, 2, 1), (0, np.pi, np.pi/2))
    rp.spin()
