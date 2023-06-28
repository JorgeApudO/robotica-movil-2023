#!/usr/bin/env python3

import rospy as rp
import numpy as np
from random import gauss, random
from copy import deepcopy

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Float64MultiArray, Int8
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry

TARGET_DEVIATION = 0.01
MAX_PARTICLES = 100
NORMAL_DISPERSION = 0.5

LOWER_ANGLE_LIMIT = -27 * np.pi / 180
UPPER_ANGLE_LIMIT = 27 * np.pi / 180

PARTICLE = 1  # sensor model and particle
WAIT = 2  # procesando info
MOVEMENT = 3  # move robot
DONE = 4  # Finish


class PFMap:
    def __init__(self):
        rp.init_node("pf_node")

        self.infered_pose = None
        self.particle_idxs = np.arange(MAX_PARTICLES)
        self.particles = np.zeros((MAX_PARTICLES, 3))
        self.weights = np.ones(MAX_PARTICLES) / float(MAX_PARTICLES)

        self.weights = None
        self.particles = None

        self.odom_data = np.zeros(3, dtype=np.float32)
        self.last_pose = np.zeros(3, dtype=np.float32)

        self.lidar_ready = False
        self.odom_ready = False
        self.map_ready = False

        # ---
        # Map loader
        rp.Subscriber("/map", OccupancyGrid, self.load_map_grid)

        # ---
        # Odometry subscriber
        rp.Subscriber("/update_odom", Odometry, self.update_odom)

        self.change_state = rp.Publisher("/state_man", Int8, queue_size=1)
        rp.loginfo("Waiting change state subscriber")
        while self.change_state.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

        self.final_pose = rp.Publisher("/final_pose", Pose, queue_size=1)
        rp.loginfo("Waiting final pose subscriber")
        while self.final_pose.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

    def compute_pose(self):
        return np.dot(self.particles, self.weights)

    def init_particles(self):
        pass

    def load_map_grid(self, data):
        self.map_info = data.info

        # 0: vacio, -1: desconocido, 100: obstaculo
        array_255 = np.array(data.data).reshape(
            (data.info.height, data.info.width))

        # 0: ocupado, 1: vacio
        self.permissible = np.zeros_like(array_255, dtype=bool)
        self.permissible[array_255 == 0] = 1
        self.map_ready = True

    def motion_model(self, movement):
        cos = np.cos(self.particles[:, 2])
        sin = np.sin(self.particles[:, 2])

        dx = cos*movement[0] - sin*movement[0] + np.random.normal(
            0.0, NORMAL_DISPERSION, MAX_PARTICLES)
        dy = cos*movement[1] + sin*movement[1] + np.random.normal(
            0.0, NORMAL_DISPERSION, MAX_PARTICLES)
        dw = movement[2] + np.random.normal(
            0.0, NORMAL_DISPERSION, MAX_PARTICLES)

        self.particles[:, 0] += dx
        self.particles[:, 1] += dy
        self.particles[:, 2] += dw

    def normalize_weights(self):
        self.weights /= np.sum(self.weights)

    def pf(self, mov, obs):
        self.resample()

        self.motion_model(mov)

        self.sensor_model(obs)

        self.normalizee_weights()

    def resample(self):
        new_particles = np.random.choice(
            self.particles, MAX_PARTICLES, p=self.weights)
        self.particles = new_particles

    def sensor_model(self, observation):
        """Update weights based on observation"""
        pass

    def update(self):
        if self.lidar_ready and self.odom_ready and self.map_ready:
            obs = np.copy(self.observation).astype(np.float32)
            mov = np.copy(self.odom_data)
            self.odom_data = np.zeros(3)

            self.pf(mov, obs)

            self.infered_pose = self.compute_pose()

            if np.std(self.particles) < TARGET_DEVIATION:  # Seudo-Codigo?
                self.change_state.publish(DONE)
                self.final_pose.publish(self.infered_pose)
            else:
                self.change_state.publish(MOVEMENT)

    def update_lidar(self, data):
        angle_min = float(data.angle_min)
        angle_max = float(data.angle_max)
        self.angle_inc = float(data.angle_increment)

        ranges = data.ranges

        index_min = int(np.ceil(
            (LOWER_ANGLE_LIMIT - angle_min) / self.angle_inc))
        index_max = len(ranges) - int(np.ceil(
            (angle_max - UPPER_ANGLE_LIMIT) / self.angle_inc))

        ranges = np.array(ranges[index_min:index_max+1])

        ranges[ranges < float(data.range_min)] = np.NaN
        ranges[ranges > float(data.range_max)] = np.NaN

        self.observation = ranges
        self.lidar_ready = True

    def update_odom(self, data):
        pos = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y])

        yaw = quaternion_to_angle(data.pose.pose.orientation)
        pose = np.array([pos[0], pos[1], yaw])

        if isinstance(self.last_pose, np.ndarray):
            rot = rotation_matrix(-self.last_pose[2])
            delta = np.array([pos - self.last_pose[:2]])
            local_delta = rot.dot(delta)

            self.odom_data = np.array(
                [local_delta[0, 0], local_delta[0, 1], yaw - self.last_pose[2]])
            self.last_pose = pose
            self.odom_ready = True
        else:
            self.last_pose = pose

        self.update()


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def quaternion_to_angle(q):
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw
