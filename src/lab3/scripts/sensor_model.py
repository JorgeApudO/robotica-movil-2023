#!/usr/bin/env python3
import rospy as rp
import numpy as np
from statistics import NormalDist
from nav_msgs.msg import OccupancyGrid



class SensorModel:

    def __init__(self, map_data, sigma = 1, zmax = None, sensor_pos = (0,0,0)):
        rp.init_node("Sensor_model")

        self.map = map_data #No se si va a ser necesario pasarlo a metros
        self.Ndist = NormalDist(mu = 0, sigma = sigma)
        self.zmax = zmax
        self.sensor_pos = sensor_pos #pose del sensor respecto al centro del robot
        #asumo que estan todos en el mismo lugar y no varia segun el laser

        self.occupied = np.array()
        #Creo q lo mejor va a ser definir un array de solo los lugares ocupados y ahi tomar el min dist


    def likelyhoodfields(self, pose, sensor):

        #asumo pose como una tupla (x, y)
        #asumo sensor como una lista de tuplas (dist, theta)
        q = 1
        for laser in sensor:

            if laser<self.zmax:

                x = pose[0] + (
                    self.sensor_pos[0]*np.cos(self.sensor_pos[2]) - self.sensor_pos[1]*np.sin(self.sensor_pos[2])
                    + laser[0]*np.cos(self.sensor_pos[2]+laser[1]))
                
                y = pose[1] + (
                    self.sensor_pos[1]*np.cos(self.sensor_pos[2]) + self.sensor_pos[0]*np.sin(self.sensor_pos[2])
                    + laser[0]*np.sin(self.sensor_pos[2]+laser[1]))
                zpos = np.array([x,y])
                dist = min(self.occupied - zpos) # pseudo codigo pa encontrar distancia minima entre los ocupados y la pose medida por el sensor.

                q = q * self.Ndist.pdf(dist)

        return q



        

    




