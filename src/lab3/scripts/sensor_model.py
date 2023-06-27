#!/usr/bin/env python3
import rospy as rp
import numpy as np
from statistics import NormalDist
from std_msgs.msg import Float64MultiArray, Int8
from geometry_msgs.msg import PoseArray


class SensorModel:

    def __init__(self, occupied, sigma=1, sensor_pos=(0, 0, 0)):
        rp.init_node("Sensor_model")

        self.occupied = occupied
        self.Ndist = NormalDist(mu=0, sigma=sigma)
        self.poses = None
        self.sensor_pos = sensor_pos  # pose del sensor respecto al centro del robot
        # asumo que estan todos en el mismo lugar y no varia segun el laser
        rp.Subscriber('/distances', Float64MultiArray,
                      self.apply_model, queue_size=2)
        rp.Subscriber('poses', PoseArray, self.set_particle_poses)

        self.probabilities_pub = rp.Publisher(
            '/probabilities', Float64MultiArray, queue_size=1)
        rp.loginfo("Waiting probabilities subscriber")
        while self.probabilities_pub.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

        self.change_state = rp.Publisher("/task_done", Int8, queue_size=1)
        rp.loginfo("Waiting change state subscriber")
        while self.change_state.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

    def apply_model(self, sensor):
        # asumo pose como Pose Msg.position
        # asumo sensor como una lista de tuplas (dist, theta)
        if not self.poses:
            return
        q_array = np.array()
        for pose in self.poses:
            pose = pose.position
            q = 1
            for laser, angle in sensor:
                if laser != np.NaN:
                    x = (pose.x +
                         self.sensor_pos[0] * np.cos(self.sensor_pos[2]) -
                         self.sensor_pos[1] * np.sin(self.sensor_pos[2])
                         + laser * np.cos(self.sensor_pos[2] + angle))

                    y = (pose.y +
                         self.sensor_pos[1] * np.cos(self.sensor_pos[2]) +
                         self.sensor_pos[0] * np.sin(self.sensor_pos[2])
                         + laser * np.sin(self.sensor_pos[2] + angle))

                    zpos = np.array([x, y])
                    dif = zpos - self.occupied
                    dists = np.linalg.norm(dif, axis=1)
                    dist = np.min(dists)

                    q = q * self.Ndist.pdf(dist)

            q_array.append(q)

            q_array = q_array/np.linalg.norm(q_array)

        self.probabilities_pub.publish(q_array)

    def set_particle_poses(self, posearray: PoseArray):
        self.poses = posearray
