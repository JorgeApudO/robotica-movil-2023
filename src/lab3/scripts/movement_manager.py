#!/usr/bin/env python3
import rospy as rp
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2 as cv


class Robot_mov_manager():
    def __init__(self):
        rp.init_node("movement_manager")
        
        # ---
        # State
        self.enable = False
        self.move = True

        # ---
        # Enable movement
        rp.Subscriber("/enable_movement", Bool, self.update_state)

        # ---
        # CONSTANTS
        self.pond_unidades = 1000
        self.dist_threshold = 0.01
        self.ang_threshold = np.pi / 180

        # [m] Distancia a la que se detiene
        self.min_front_dist = 0.4*self.pond_unidades

        # ---
        # INITIAL CONDITIONS
        self.distance = np.array((0.0, 0.0, 0.80*self.pond_unidades))
        self.ang = 0.0
        self.vel = 0.1
        self.ang_vel = 0.0
        self.localized = False

        # Lidar data
        rp.Subscriber("/scan", LaserScan, self.update_dist)
        # ---
        # VEL PUBLISHER NODE
        self.vel_applier = rp.Publisher("yocs_cmd_vel_mux/input/navigation",
                                        Twist, queue_size=1)

        # ---
        # LOCALIZATION CONFIRMATION
        self.localized_sub = rp.Subscriber(
            "localization", Float64, self.manager)

        # ---
        # CONTROL NODE
        self.ang_set_point = rp.Publisher('/angle/setpoint',
                                          Float64, queue_size=1)
        rp.loginfo("Waiting angle distance pid setpoint process")
        while self.ang_set_point.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

        self.ang_state = rp.Publisher('/angle/state',
                                      Float64, queue_size=1)
        rp.loginfo("Waiting angle distance pid state process")
        while self.ang_state.get_num_connections() == 0 and not rp.is_shutdown():
            rp.sleep(0.1)

        self.ang_actuation = rp.Subscriber('/angle/control_effort',
                                           Float64, self.ang_actuation_fn)
        #Sigamos un muro ahora, mas facil girar 90 grados                    
        self.ang_set_point.publish(0.5*self.pond_unidades)
    
    def update_dist(self,data):
        #nose como viene la data del lidar

        #sacar 3 puntos, izq, centro y der, igual que antes.
        self.rotate = centro < self.min_front_dist

        self.distances = np.array(izq, der, centro)
        pass

    def ang_actuation_fn(self, data: Float64):
        if self.localized:
            self.vel = 0
            self.ang_vel = 0
        
        if self.rotate:
            self.ang_vel = 1
            self.vel = 0 

        if self.move:
            self.vel = 0.2
            self.ang_vel = data

        self.publish_vel()

    def publish_depth(self, data):
        # Publish difference between left and right distance
        if self.show_img is not None:
            cv.imshow("out", self.show_img)
        if not self.arrow_rotation:
            dif_distance = self.distance[1]
            self.ang_state.publish(dif_distance)

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

    def manager(self, data: Float64):

        if not data:
            self.vel = 0
            self.ang_vel = 0
            self.localized = True
            self.notify()
        else:
            self.move = True

    def notify(self):
        # Avisar por los parlantes y todo eso
        pass

    def update_state(self, data):
        if data.data != self.enable:
            self.enable = not self.enable


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


if __name__ == "__main__":
    robot_mov = Robot_mov_manager()
    rospy.spin()
