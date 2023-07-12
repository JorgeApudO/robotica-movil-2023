#!/usr/bin/env python3

import rospy as rp
import actionlib
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped


def get_move_base_goal(pose):
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = 'map'
    move_base_goal.target_pose.header.stamp = rp.Time.now()
    move_base_goal.target_pose.pose = pose
    return move_base_goal


class NavStack:
    def __init__(self):
        rp.init_node('nav_stack')

        self.wait = True
        rp.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                      self.pose_update, queue_size=1)

        self.move_base_client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.position_cov = PoseWithCovarianceStamped()

        self.pose_queue = []

    def pose_update(self, data):
        self.position_cov = data

    def goal_done(self, status, result):
        rp.loginfo(
            f"DONE status: {self.move_base_client.get_goal_status_text()} " +
            f"({str(status)})")

    def add_goal(self, x, y, a):
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.orientation = Quaternion(*quaternion_from_euler(0, 0, a))

        self.pose_queue.append(goal)

    def rotate(self):
        first = True

        while first or cov_x > 1 or cov_y > 1:
            ang = euler_from_quaternion(
                (self.position_cov.pose.pose.orientation.x,
                 self.position_cov.pose.pose.orientation.y,
                 self.position_cov.pose.pose.orientation.w,
                 self.position_cov.pose.pose.orientation.z))[2]
            ang += np.pi/18  # 5 grados
            move_base_goal = get_move_base_goal(self.position_cov.pose.pose)
            move_base_goal.target_pose.pose.orientation = Quaternion(
                *quaternion_from_euler(0, 0, ang))

            self.move_base_client.send_goal(move_base_goal)
            if first:
                self.move_base_client.wait_for_result(rp.Duration(15))
                first = False
            else:
                self.move_base_client.wait_for_result(rp.Duration(5))

            cov_x = self.position_cov.pose.covariance[0]
            cov_y = self.position_cov.pose.covariance[7]
            rp.loginfo(f"cov x: {cov_x}, cov y: {cov_y}")

    def main_loop(self):
        while not rp.is_shutdown() and self.pose_queue:
            goal = get_move_base_goal(self.pose_queue.pop(0))
            self.move_base_client.send_goal(goal, done_cb=self.goal_done)
            self.move_base_client.wait_for_result()


if __name__ == "__main__":
    nav_stack = NavStack()

    nav_stack.rotate()
    nav_stack.add_goal(25, 1, 0)
    nav_stack.add_goal(27, 12, np.pi/2)
    nav_stack.add_goal(7, 15, 3*np.pi/4)
    nav_stack.main_loop()
