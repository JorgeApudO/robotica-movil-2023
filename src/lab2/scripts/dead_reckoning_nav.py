#!/usr/bin/env python3

import rospy as rp
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import numpy as np


def corregir_angulo(ang):
    if ang > np.pi:
        return np.pi - ang
    elif ang < -np.pi:
        return (2 * np.pi) + ang
    else:
        return ang


class Movement(object):
    def __init__(self) -> None:
        self._v = 0.2  # m/s
        self._v_ang = 1  # rads/s
        rp.init_node("Movement_manager")
        self.vel_applier = rp.Publisher("yocs_cmd_vel_mux/input/navigation",
                                        Twist, queue_size=10)
        self.rate_hz = 10
        self.rate = rp.Rate(self.rate_hz)
        self.mov_sub = rp.Subscriber("goal_list", PoseArray,
                                     self.accion_mover)
        self.current_pose = Pose()
        self.real_pose_sub = rp.Subscriber("real_pose", Pose,
                                           self.real_pose_manager)
        self.real_pose = Pose()
        self.odom_pose_sub = rp.Subscriber("odom_pose", Odometry,
                                           self.odom_pose_manager)
        self.odom_pose = Pose()

    @property
    def v(self):
        return self._v

    @property
    def v_ang(self):
        return self._v_ang

    def real_pose_manager(self, data):
        self.real_pose = data

    def odom_pose_manager(self, data):
        self.odom_pose = data.PoseWithCovariance.Pose()

    def aplicar_velocidad(self, speed_command: list):
        for mov in speed_command:
            speed = Twist()
            speed.linear.x, speed.angular.z = mov[0], mov[1]
            self.vel_applier.publish(speed)
            self.rate.sleep()

    def mover_robot_a_destino(self, goal_pose: Pose):
        x_goal, y_goal = goal_pose.position.x, goal_pose.position.y
        goal_pos = np.array((x_goal, y_goal))
        yaw_goal = euler_from_quaternion((goal_pose.orientation.x,
                                          goal_pose.orientation.y,
                                          goal_pose.orientation.z,
                                          goal_pose.orientation.w))[2]
        x_initial = self.current_pose.position.x
        y_initial = self.current_pose.position.y
        initial_pos = np.array((x_initial, y_initial))
        yaw_initial = euler_from_quaternion((self.current_pose.orientation.x,
                                             self.current_pose.orientation.y,
                                             self.current_pose.orientation.z,
                                             self.current_pose.orientation.w))[2]
        yaw_initial = corregir_angulo(yaw_initial)

        instructions = []

        # align yaw
        ang_aplicado = corregir_angulo(yaw_goal - yaw_initial)
        direccion_giro = -1 if ang_aplicado > 0 else 1
        instructions.append((0, self.v_ang * direccion_giro,
                             abs(ang_aplicado) / self.v_ang))

        # move
        distance = np.linalg.norm(goal_pos - initial_pos)
        instructions.append((self.v, 0, distance / self.v))

        self.aplicar_velocidad(instructions)
        self.current_pose = goal_pose

    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            self.mover_robot_a_destino(pose)
            time.sleep(1)


if __name__ == "__main__":
    time.sleep(1)
    m = Movement()
    rp.spin()
