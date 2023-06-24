#!/usr/bin/env python3
import rospy as rp
import numpy as np
from statistics import NormalDist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose


class SensorModel:

    def __init__(self, occupied, sigma = 1, zmax = None, sensor_pos = (0,0,0)):
        rp.init_node("Sensor_model")

        self.occupied = occupied
        self.Ndist = NormalDist(mu = 0, sigma = sigma)
        self.zmax = zmax
        self.sensor_pos = sensor_pos #pose del sensor respecto al centro del robot
        #asumo que estan todos en el mismo lugar y no varia segun el laser
        self.measurement_sub = rp.Subscriber('distances', Float64MultiArray, self.apply_model, 
                                    queue_size = 2)
        self.pose_sub = rp.Subscriber('position',Pose ,self.get_pose,queue_size = 2)
        
    def occupied_data(self):
        pass 

    
    def apply_model(self, data):
        self.likelyhoodfields(data)

    
    def get_pose(self, data):
        self.pose = data.position


    def likelyhoodfields(self, sensor):
        
        #asumo pose como Pose Msg.position
        
        #asumo sensor como una lista de tuplas (dist, theta)
        q = 1
        for laser in sensor:

            if laser<self.zmax:

                x = self.pose.x + (
                    self.sensor_pos[0]*np.cos(self.sensor_pos[2]) - self.sensor_pos[1]*np.sin(self.sensor_pos[2])
                    + laser[0]*np.cos(self.sensor_pos[2]+laser[1]))
                
                y = self.pose.y + (
                    self.sensor_pos[1]*np.cos(self.sensor_pos[2]) + self.sensor_pos[0]*np.sin(self.sensor_pos[2])
                    + laser[0]*np.sin(self.sensor_pos[2]+laser[1]))
                
                zpos = np.array([x,y])
                dif = zpos - self.occupied
                dists = np.linalg.norm(dif, axis = 1)
                dist = np.min(dists)

                q = q * self.Ndist.pdf(dist)

        return q


        

    




