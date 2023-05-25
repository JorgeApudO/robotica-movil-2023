#!/usr/bin/env python3

import rospy as rp
from geometry_msgs.msg import Twist, PoseArray, Pose, Float64
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
        self._v = 0.0  # m/s
        self._v_ang = 0.0  # rads/s
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

        self.pid_set_point_dist = rp.Publisher('/reckoning_dist/setpoint',
                                               Float64, queue_size=1)
        while self.pid_set_point_dist.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.2)

        self.dist_state = rp.Publisher('/reckoning_dist/state',
                                       Float64, queue_size=1)
        while self.dist_state.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.2)

        self.actuation_dist = rp.Subscriber('/reckoning_dist/control_effort',
                                            Float64, self.dist_actuation)

        self.period = 1 / self.rate_hz
        rp.Timer(rp.Duration(self.period), self.get_dist_odom)

    @property
    def v(self):
        return self._v

    @property
    def v_ang(self):
        return self._v_ang

    def dist_actuation(self, data):
        self._v = float(data.data)
        rp.loginfo(f"Speed received: {self.v}")

    def get_dist_odom(self, data):
        # Get odometry and update distance to objective
        pass

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

    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            pass


if __name__ == "__main__":
    time.sleep(1)
    m = Movement()
    rp.spin()
