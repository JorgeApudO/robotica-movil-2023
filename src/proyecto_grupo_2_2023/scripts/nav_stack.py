#!/usr/bin/env python3

import rospy as rp
import actionlib
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped


class NavStack:
    def __init__(self):
        rp.init_node('nav_stack')

        self.wait = True
        rp.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.init_pos_setter ,queue_size=1)

        self.move_base_client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def init_pos_setter(self, data):
        if self.wait:
            rp.loginfo(f"inital pose setting")
            initial_pose = data.pose.pose

            self.initial_pos = Pose()

            self.initial_pos.position.x = initial_pose.position.x
            self.initial_pos.position.y = initial_pose.position.y
            self.initial_pos.orientation = initial_pose.orientation

            self.wait = False
        

    def goal_done(self, status, result):
        rp.loginfo(
            f"DONE status: {self.move_base_client.get_goal_status_text()} " +
            f"({str(status)})")

    def set_goals(self, list_x, list_y, list_a):

        for i in range(2):
            ang = euler_from_quaternion((self.initial_pos.orientation.x,
                                        self.initial_pos.orientation.y,
                                        self.initial_pos.orientation.w,
                                        self.initial_pos.orientation.z
                                        ))[2] + i*(np.pi)/2
            list_x.insert(0, self.initial_pos.position.x)
            list_y.insert(0, self.initial_pos.position.y)
            list_a.insert(0, ang)
        print(list_x)
        print(list_y)
        print(list_a)
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
            rp.loginfo(f"Arrived at: (x:{x}, y:{y}, yaw:{a})")


if __name__ == "__main__":
    nav_stack = NavStack()

    rp.loginfo("Waiting for initial pose")
    while nav_stack.wait:
        rp.sleep(0.1)
    
    nav_stack.set_goals([1, 1, 3], [1, 2, 1], [0, np.pi, np.pi/2])
    rp.spin()
