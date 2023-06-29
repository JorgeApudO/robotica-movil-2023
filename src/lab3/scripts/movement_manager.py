#!/usr/bin/env python3
import rospy as rp
from std_msgs.msg import Float64, Bool, Float64MultiArray, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2 as cv


PARTICLE = 1  # sensor model and particle
WAIT = 2  # procesando info
MOVEMENT = 3  # move robot
DONE = 4  # Finish

LOWER_ANGLE_LIMIT = -27 * np.pi / 180
UPPER_ANGLE_LIMIT = 27 * np.pi / 180


class Robot_mov_manager():
    def __init__(self):
        rp.init_node("movement_manager")
        # ---
        # State
        self.enable = False
        # ---

        # Enable movement
        rp.Subscriber("/enable_movement", Bool, self.update_state)
        # ---

        # Change state
        self.change_state = rp.Publisher("/state_man", Int8, queue_size=1)
        # ---

        # CONSTANTS
        self.pond_unidades = 1
        self.min_front_dist = 0.2*self.pond_unidades
        # ---

        # INITIAL CONDITIONS
        self.distance = np.array(
            (0.4*self.pond_unidades, 0.8*self.pond_unidades))
        self.vel = 0.1
        self.ang_vel = 0.0
        self.control_cont = 0
        # ---

        # LIDAR
        rp.Subscriber("/depth_data", LaserScan, self.update_dist)
        # ---

        # VEL PUBLISHER NODE
        self.vel_applier = rp.Publisher("yocs_cmd_vel_mux/input/navigation",
                                        Twist, queue_size=1)
        # ---

        # CONTROL NODE
        self.ang_set_point = rp.Publisher('/angle/setpoint',
                                          Float64, queue_size=1)

        rp.loginfo("Waiting angle distance pid setpoint process")
        while self.ang_set_point.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)
        rp.loginfo("Ready angle distance pid setpoint process")

        self.ang_state = rp.Publisher('/angle/state',
                                      Float64, queue_size=1)
        rp.loginfo("Waiting angle distance pid state process")
        while self.ang_state.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)
        rp.loginfo("Ready angle distance pid state process")

        self.ang_actuation = rp.Subscriber('/angle/control_effort',
                                           Float64, self.ang_actuation_fn)
        self.ang_set_point.publish(0.5*self.pond_unidades)
        # ---

        # HIGH LEVEL MOVEMENT CONTROL!
        rp.Timer(rp.Duration(0.1), self.control)

    def update_dist(self, data):
        data = lidar_manage(data)
        izq, centro = get_distances(data)
        self.rotate = centro < self.min_front_dist
        self.distances = np.array(izq, centro)
        self.ang_state.publish(izq)

    def ang_actuation_fn(self, data: Float64):
        if self.enable:
            if self.rotate:
                self.ang_vel = 1
                self.vel = 0
            else:
                self.vel = 0.1
                self.ang_vel = float(data.data)
        else:
            self.reset()
        self.publish_vel()

    def publish_vel(self):
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

    def update_state(self, data):
        self.enable = data.data

    def control(self, data):
        if self.enable:
            self.control_cont += 1
            if self.control_cont == 15:  # 1.5 segs
                self.change_state.publish(PARTICLE)
                self.reset()
        else:
            self.reset()

    def reset(self):
        self.vel = 0
        self.ang_vel = 0
        self.enable = False
        self.control_cont = 0


def sawtooth(rad):
    return (rad - np.pi) % (2*np.pi) - np.pi


def min_rotation_diff(goal, actual):
    if abs(goal - actual) > np.pi:
        if actual < 0:
            return 2*np.pi + actual - goal
        else:
            return actual - goal - 2*np.pi
    else:
        return actual - goal


def get_distances(data):
    izq = np.array([np.nan,0])
    contador = 0
    while np.isnan(izq[0]):
        izq = data[contador]
        contador += 1
    # ---
    centro = np.array([0, np.inf])
    contador = 0
    while centro[1] >= 5 * (np.pi / 180):
        centro = data[contador]
        contador += 1

    izq = izq[0]*abs(np.sin(izq[1]))
    centro = centro[0]*abs(np.sin(centro[1]))
    rp.loginfo(f"izq:{izq}, centro:{centro}")
    return izq, centro

def lidar_manage(data):
    angle_min,angle_max, angle_inc = float(data.angle_min), float(data.angle_max), \
                                                                float(data.angle_increment)
    ranges = data.ranges
    index_min = int(np.ceil((LOWER_ANGLE_LIMIT - angle_min) / angle_inc))
    index_max = len(ranges) - int(np.ceil((angle_max - UPPER_ANGLE_LIMIT) / angle_inc))

    ranges = np.array(ranges[index_min:index_max+1])
    ranges[ranges < float(data.range_min)] = np.NaN
    ranges[ranges > float(data.range_max)] = np.NaN
    
    lidar_info = np.array([[x, LOWER_ANGLE_LIMIT + i*angle_inc]
                              for i, x in enumerate(ranges)])
    return lidar_info

if __name__ == "__main__":
    robot_mov = Robot_mov_manager()
    rp.spin()
