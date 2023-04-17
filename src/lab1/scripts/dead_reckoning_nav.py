#!/usr/bin/env python3

import rospy as rp
from geometry_msgs.msg import Twist, PoseArray, Pose
from tf.transformations import euler_from_quaternion
import time
import math


class Movement(object):
    def __init__(self) -> None:
        self.v = 0.2  # m/s
        self.v_ang = 1  # rads/s
        rp.init_node("Movement_manager")
        self.vel_applier = rp.Publisher("yocs_cmd_vel_mux/input/navigation", Twist, queue_size=10)
        self.rate_hz = 10
        self.rate = rp.Rate(self.rate_hz)
        self.mov_sub = rp.Subscriber("goal_list", PoseArray,
                                     self.accion_mover)  
        self.current_pose = Pose()
        self.real_pose_sub=rp.Subscriber("real_pose", Pose, self.real_pose_manager)
        self.real_pose=Pose()
        self.correcciones=[]


    def real_pose_manager(self, data):

        self.real_pose=data

    def aplicar_velocidad(self, speed_command: list):
        for mov in speed_command:
            inicial_time = time.time()
            angulo_real_inicial=euler_from_quaternion((self.real_pose.orientation.x, self.real_pose.orientation.y,
                                         self.real_pose.orientation.z, self.real_pose.orientation.w))[2]
            while time.time() <= inicial_time+mov[2]:
                
                speed = Twist()
                speed.linear.x, speed.angular.z = mov[0], mov[1]
                self.vel_applier.publish(speed)
                self.rate.sleep()


            angulo_real_final=euler_from_quaternion((self.real_pose.orientation.x, self.real_pose.orientation.y,
                                         self.real_pose.orientation.z, self.real_pose.orientation.w))[2]
            rotacion_real=angulo_real_final - angulo_real_inicial
            rotacion_esperada=mov[1]*mov[2]
            if mov[1]!=0:
                error=rotacion_esperada-rotacion_real
                t_faltante=error/mov[1]
                self.correcciones.append((mov[2]+t_faltante)/mov[2])

        rp.loginfo(f"movement_done")
        if len(self.correcciones)!=0:
            rp.loginfo(f"{sum(self.correcciones)/len(self.correcciones)}")



                

    def mover_robot_a_destino(self, goal_pose: Pose): 
        x_goal, y_goal = goal_pose.position.x, goal_pose.position.y
        yaw_goal = euler_from_quaternion((goal_pose.orientation.x, goal_pose.orientation.y,
                                         goal_pose.orientation.z, goal_pose.orientation.w))[2]
        instructions = []  
        x_initial, y_initial = self.current_pose.position.x, self.current_pose.position.y
        yaw_initial = euler_from_quaternion((self.current_pose.orientation.x, self.current_pose.orientation.y,
                                            self.current_pose.orientation.z, self.current_pose.orientation.w))[2]
        
        rp.loginfo(f"{x_goal}, {y_goal}, {yaw_goal}")
        # align x
        if x_initial-x_goal !=0:
            ang = (0 if x_goal - x_initial >= 0 else math.pi)
            dir_ang = (-1 if yaw_initial - ang > 0 else 1)
            if yaw_initial-ang != 0:
                instructions.append((0, self.v_ang * dir_ang, abs(ang - yaw_initial) / self.v_ang))
            instructions.append((self.v, 0, abs(x_initial - x_goal) / self.v))
            yaw_initial = ang


        # align y
        if y_initial-y_goal != 0:
            
            ang = (math.pi/2 if y_goal - y_initial > 0 else -math.pi/2)
            dir_ang = (-1 if yaw_initial - ang > 0 else 1)  
            if yaw_initial-ang != 0:
                instructions.append((0, self.v_ang * dir_ang, abs(ang - yaw_initial) / self.v_ang))
            instructions.append((self.v, 0, abs(y_initial - y_goal) / self.v))
            yaw_initial = ang

        # yaw align
        
        
        if yaw_initial-yaw_goal != 0:
            dir_ang = (-1 if yaw_initial - yaw_goal > 0 else 1)
            instructions.append((0, self.v_ang * dir_ang, abs(yaw_goal - yaw_initial) / self.v_ang))

        self.aplicar_velocidad(instructions)
        self.current_pose = goal_pose
        
    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            self.mover_robot_a_destino(pose)
            time.sleep(1)

if __name__ == "__main__":
    m = Movement()
    rp.spin()
