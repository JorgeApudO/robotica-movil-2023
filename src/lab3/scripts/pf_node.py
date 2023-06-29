#!/usr/bin/env python3

import rospy as rp
import numpy as np
from random import gauss, random
from copy import deepcopy
from scipy.stats import norm

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Float64MultiArray, Int8, Header
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from sklearn.neighbors import KDTree

TARGET_DEVIATION = 0.01
MAX_PARTICLES = 100
NORMAL_DISPERSION = 0.01
SENSOR_DISPERSION = 0.5

LOWER_ANGLE_LIMIT = -27 * np.pi / 180
UPPER_ANGLE_LIMIT = 27 * np.pi / 180

PARTICLE = 1  # sensor model and particle
WAIT = 2  # procesando info
MOVEMENT = 3  # move robot

class PFMap:
    def __init__(self):
        rp.init_node("pf_node")

        self.flip = bool(rp.get_param('/localization/flip_map', 0))

        self.infered_pose = None
        self.particle_idxs = np.arange(MAX_PARTICLES)
        self.particles = np.zeros((MAX_PARTICLES, 3))
        self.weights = np.ones(MAX_PARTICLES) / float(MAX_PARTICLES)

        # self.weights = None
        # self.particles = None

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

        # ---
        # Lidar subscriber
        rp.Subscriber('/distances', LaserScan, self.update_lidar)

        self.change_state = rp.Publisher("/state_man", Int8, queue_size=1)
        rp.loginfo("Waiting change state subscriber")
        while self.change_state.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)
        rp.loginfo("Ready change state subscriber")

        self.final_pose = rp.Publisher("/final_pose", Pose, queue_size=1)
        rp.loginfo("Waiting final pose subscriber")
        while self.final_pose.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

        # ---
        # Particles rviz
        self.rviz_pub = rp.Publisher('/rviz/particles', PoseArray, queue_size=1)
        self.pose_pub = rp.Publisher('/rviz/position', PoseStamped, queue_size=1)

        rp.loginfo("Ready final pose subscriber")

    def compute_pose(self):
        return np.dot(self.particles, np.repeat(self.weights, 3, axis=1).T)

    def init_particles(self):
        x, y = np.where(self.map == 1)
        indices = np.random.randint(0, len(x), size=MAX_PARTICLES)

        states = np.zeros((MAX_PARTICLES, 3))
        states[:,0] = y[indices]
        rp.loginfo(f"{y[indices]}")
        states[:,1] = x[indices]
        rp.loginfo(f"{x[indices]}")
        states[:,2] = np.random.random(MAX_PARTICLES) * np.pi * 2

        map_to_world(states, self.map_info)
        self.particles = states
        self.weights[:] = 1.0 / MAX_PARTICLES
        self.publish_particles()
        rp.loginfo("PARTICLES INITIALIZED")

    def load_map_grid(self, data):
        self.map_info = data.info
        self.resolution = self.map_info.resolution
        self.origin = self.map_info.origin

        # 0: vacio, -1: desconocido, 100: obstaculo
        array_255 = np.array(data.data).reshape(
            (data.info.height, data.info.width))

        if self.flip:
            array_255 = np.flip(array_255, axis=0)

        # 0: ocupado, 1: vacio
        self.map = np.zeros_like(array_255, dtype=bool)
        self.map[array_255 == 0] = 1

        global_indices = np.array(list(np.ndindex(*self.map.shape)))
        self.global_coord = np.apply_along_axis(
            self.cell_position, 1, global_indices)

        indices_0 = np.transpose((self.map == 0).nonzero())
        coord_0 = np.apply_along_axis(self.cell_position, 1, indices_0)
        rp.loginfo(coord_0.shape) #[[x,y],[x2,y2].....] <-[x*,y*]
        self.occupied = KDTree(coord_0)
        self.map_ready = True

        rp.loginfo("MAP LOADED")
        self.init_particles()
        self.update()

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

        self.normalize_weights()
    
    def publish_particles(self):
        if isinstance(self.infered_pose, np.ndarray):
            ps = PoseStamped()
            ps.pose.position.x = self.infered_pose[0]
            ps.pose.position.y = self.infered_pose[1]
            ps.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.infered_pose[2]))
            self.pose_pub.publish(ps)

        pa = PoseArray()
        pa.poses = list(map(particle_to_pose, self.particles))

        header = Header()
        header.stamp = rp.Time.now()
        header.frame_id = "map"

        pa.header = header
        self.rviz_pub.publish(pa)

    def resample(self):
        # rp.loginfo(f"{self.particles.shape}")
        # rp.loginfo(f"{self.weights.shape}")
        rp.loginfo(f"weights: {self.weights}")
        new_particles_idx = np.random.choice(
            self.particles.shape[0], MAX_PARTICLES, p=self.weights)

        self.particles = self.particles[new_particles_idx]

    def sensor_model(self, observation):

        q_array = list()
        for particula in self.particles:
            prob_array = list()
            for laser, angle in observation:
                if not np.isnan(laser):
                    # Tomamos la ubicacion del robot como la ubicacion del lidar
                    x = (particula[0] + laser * np.cos(particula[2] + angle))
                    y = (particula[1] + laser * np.sin(particula[2] + angle))
                    x = (((x - (self.origin.position.x)) /
                         self.resolution) - 0.5) // 1
                    y = (((y - (self.origin.position.y)) /
                         self.resolution) - 0.5) // 1
                    min_dist_occupied = self.occupied.query([[x,y]])[0]/1000
                    prob_pos = (1/(np.sqrt(2*np.pi)*SENSOR_DISPERSION)) * np.exp( - (min_dist_occupied ** 2) / (2 * SENSOR_DISPERSION**2))
                    prob_array.append(*prob_pos)
            q = sum(prob_array) / len(prob_pos)
            q_array.append(q)
        q_array = np.array(q_array)
        q_array /= np.sum(q_array)
        self.weights = q_array

    def update(self):
        if self.lidar_ready and self.odom_ready and self.map_ready:
            obs = np.copy(self.observation).astype(np.float32) 
            mov = np.copy(self.odom_data)
            self.odom_data = np.zeros(3)

            self.pf(mov, obs)

            infered_pose = self.compute_pose()
            self.infered_pose = Pose()
            self.infered_pose.position.x = infered_pose[0]
            self.infered_pose.position.y = infered_pose[1]
            self.infered_pose.orientation = infered_pose[2]

            if np.std(self.particles) < TARGET_DEVIATION:
                self.change_state.publish(WAIT)
                self.final_pose.publish(self.infered_pose)
            else:
                self.change_state.publish(MOVEMENT)

            self.odom_ready = False
            self.lidar_ready = False
        
        self.publish_particles()

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

        lidar_info = np.array([[x, LOWER_ANGLE_LIMIT + i*self.angle_inc]
                              for i, x in enumerate(ranges)])

        self.observation = lidar_info
        self.lidar_ready = True

        self.update()

    def update_odom(self, data):
        pos = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y])

        yaw = quaternion_to_angle(data.pose.pose.orientation)
        pose = np.array([pos[0], pos[1], yaw])

    
        rot = rotation_matrix(-self.last_pose[2])
        delta = np.array([pos - self.last_pose[:2]])
        local_delta = rot.dot(delta.T).T

        self.odom_data = np.array(
            [local_delta[0, 0], local_delta[0, 1], yaw - self.last_pose[2]])
        self.last_pose = pose
        self.odom_ready = True

        self.update()

    def cell_position(self, pos):
        x = (pos[0] + 0.5) * self.resolution + self.origin.position.x
        y = (pos[1] + 0.5) * self.resolution + self.origin.position.y
        return np.array((x, y))


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def quaternion_to_angle(q):
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw

def map_to_world(poses, map_info):
    res = map_info.resolution
    ang = quaternion_to_angle(map_info.origin.orientation)

    cos, sin = np.cos(ang), np.sin(ang)

    tmp = np.copy(poses[:,0])
    poses[:,0] = cos * poses[:,0] - sin * poses[:,1]
    poses[:,1] = sin * tmp + cos * poses[:,1]

    poses[:,:2] *= float(res)

    poses[:,0] += map_info.origin.position.x
    poses[:,1] += map_info.origin.position.y
    poses[:,2] += ang

def particle_to_pose(particle):
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = Quaternion(*quaternion_from_euler(0, 0, particle[2]))
    return pose

# Basado
if __name__ == "__main__":
    robot = PFMap()
    rp.spin()
