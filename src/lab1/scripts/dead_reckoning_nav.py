#!/usr/bin/env python

import rospy as rp
from geometry_msgs.msg import Twist, PoseArray

import time
import math
#importar topicos


class Movement(object):
    def __init__(self) -> None:
        self.v = 0.2 # m/s
        self.ang = 1 #rads/s
        rp.init_node("Movement_manager")
        self.vel_applier = rp.Publisher("yocs_cmd_vel_mux/input/navigation",
                Twist, queue_size = 10)
        self.rate_hz = 10
        self.rate = rp.Rate(self.rate_hz)
        self.mov_sub= rp.Subscriber("goal_list",PoseArray,self.accion_mover) 
        self.init_pose = (0,0,0)


    def aplicar_velocidad(self,speed_command: list ):
        for mov in speed_command:
            inicial_time = time.time()
            while time.time()<= inicial_time+mov[2]:
                speed= Twist()
                speed.linear.x, speed.angular.z= mov[0],mov[1]            
                rp.loginfo("linear speed:", mov[0],"angular speed:",mov[1])
                self.vel_applier.publish(speed)
                self.rate.sleep()

    def mover_robot_a_destino(self, goal_pose:tuple): #EDITAR POSEARRAY
        x_movement, y_movement, t_movement = goal_pose[0], goal_pose[1], goal_pose[2]
        
        intructions = [( self.v, 0, x_movement / self.v ), ( 0, self.ang, math.pi / self.ang ),
                        ( self.v, 0, y_movement / self.v ), ( 0, self.ang, t_movement / self.v )]
        self.aplicar_velocidad(intructions)

        
    def accion_mover(self,pose_list:list):
        for pose in pose_list:
            self.mover_robot_a_destino(pose)




if __name__ == "__main__":
    mov = Movement()
    rp.spin()
