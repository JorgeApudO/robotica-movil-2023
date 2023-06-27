#!/usr/bin/env python3

import rospy as rp
import numpy as np
from random import gauss, random
from copy import deepcopy

from std_msgs.msg import Float64MultiArray, Int8

TARGET_DEVIATION = 0.01

PARTICLE = 1 # sensor model and particle 
WAIT = 2 # procesando info
MOVEMENT = 3 # move robot
DONE = 4 #Finish

class PFMap:
    def __init__(self):
        rp.init_node("pf_node")

        self.weights = None
        self.particles = None

        rp.Subscriber("/probabilities", Float64MultiArray, self.update_weights)

        self.change_state = rp.Publisher("/task_done", Int8, queue_size=1)
        self.final_pose = rp.Publisher("/final_pose",Float64MultiArray, queue_size=1)
        
        self.robot_position = np.array((0, 0))
        self.robot_angle = 0.0

    def update_robot_position(self, x, y, w):
        dist = np.linalg.norm(np.array((x, y)) - self.robot_position)
        self.robot_position = np.array((x, y))

        turn = w - self.robot_angle
        self.robot_angle = w

        self.move(turn, dist)
    
    def update_weights(self, weights):
        self.weights = np.array(weights)
        self.resample()

        if np.std(self.particles) < TARGET_DEVIATION: # Seudo-Codigo?
            self.change_state.publish(DONE)
            self.final_pose.publish(pose) # get pose by some method
        else:
            self.change_state.publish(MOVEMENT)
    
    def move(self, turn_angle, forward_distance):
        for particle in self.particles:
            particle.move(turn_angle, forward_distance)

    def resample(self):
        normalized_prob = np.array(self.weights) / sum(self.weights)
        cumulative_prob = np.array()
        current = 0
        nop = len(self.particles)

        for i in range(nop):
            cumulative_prob.append(current)
            current += normalized_prob[i]
        
        resampled = np.array()

        for i in range(nop):
            curr = random()
            j = nop - 1
            
            while cumulative_prob[j] > curr:
                j -= 1
            
            resampled.append(deepcopy(self.particles[j]))

        self.particles = resampled
        

class Particle:
    def __init__(self, x, y, w, noises=(0.0, 0.0, 0.0)):
        self.x = x
        self.y = y
        self.w = w

        self.set_noises(*noises) 
    
    def set_noises(self, wn, dn, sn):
        self.turn_noise = wn
        self.move_noise = dn
        self.snese_noise = sn

    def set_position(self, x, y, w):
        w %= (2 * np.pi)
        self.x, self.y, self.w = x, y, w
    
    def rotate(self, dw):
        self.w += (dw + gauss(0, self.turn_noise))
        self.w %= 2*np.pi
    
    def forward(self, dd):
        dist = dd + gauss(0, self.move_noise)
        self.x += dist * np.cos(self.w)
        self.y += dist * np.sin(self.w)
    
    def move(self, angle, distance):
        self.turn(angle)
        self.forward(distance)