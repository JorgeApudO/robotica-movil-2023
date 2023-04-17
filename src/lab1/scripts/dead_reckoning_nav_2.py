#!/usr/bin/env python3

import rospy as rp
from geometry_msgs.msg import Twist, PoseArray, Pose, Vector3
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
                                     self.accion_mover)  # investigar PoseArray
        self.current_pose = Pose()
        self.obs_sub = rp.Subscriber('/occupancy_state', Vector3, self.update_obstacles)
        self.obs = (0,0,0)
        self.movimiento = True

    def update_obstacles(self, data: Vector3):
        self.obs = (data.x, data.y, data.z)

    def aplicar_velocidad(self, speed_command: list):
        for mov in speed_command:
            inicial_time = time.time()
            elapsed_time = 0
            while elapsed_time <= mov[2]:
                while any(self.obs):
                    if self.movimiento:
                        #Hace ruido
                        speed = Twist()
                    self.vel_applier.publish(speed)
                    self.movimiento = False

                else:
                    self.movimiento = True
                    speed = Twist()
                    speed.linear.x, speed.angular.z = mov[0], mov[1]
                    self.vel_applier.publish(speed)
                    self.rate.sleep()
                    elapsed_time += time.time() - inicial_time
                inicial_time = time.time()

    def mover_robot_a_destino(self, goal_pose: Pose):  # EDITAR POSEARRAY
        x_goal, y_goal = goal_pose.position.x, goal_pose.position.y
        yaw_goal = euler_from_quaternion((goal_pose.orientation.x, goal_pose.orientation.y,
                                         goal_pose.orientation.z, goal_pose.orientation.w))[2]
        instructions = []  # Hay que definir esto bien
        x_initial, y_initial = self.current_pose.position.x, self.current_pose.position.y
        yaw_initial = euler_from_quaternion((self.current_pose.orientation.x, self.current_pose.orientation.y,
                                            self.current_pose.orientation.z, self.current_pose.orientation.w))[2]
        # align x
        rp.loginfo(f"initial: {x_initial},{y_initial},{yaw_initial}")
        rp.loginfo(f"goal: {x_goal},{y_goal},{yaw_goal}")
        ang = (0 if x_goal - x_initial >= 0 else math.pi)
        dir_ang = (-1 if yaw_initial - ang > 0 else 1)
        if yaw_initial-ang != 0:
            instructions.append((0, self.v_ang * dir_ang, abs(ang - yaw_initial) / self.v_ang))
            rp.loginfo(f"""se alineo angular x""")

        if x_initial-x_goal !=0:
            instructions.append((self.v, 0, abs(x_initial - x_goal) / self.v))
            rp.loginfo(f"""se movio linear x""")


        # align y
        yaw_initial = ang
        ang = (math.pi/2 if y_goal - y_initial > 0 else -math.pi/2)
        dir_ang = (-1 if yaw_initial - ang > 0 else 1)  # sign?
        if yaw_initial-ang != 0:
            rp.loginfo(f"""se alineo angular y""")
            instructions.append((0, self.v_ang * dir_ang, abs(ang - yaw_initial) / self.v_ang))
        if y_initial-y_goal != 0:
            instructions.append((self.v, 0, abs(y_initial - y_goal) / self.v))
            rp.loginfo(f"""se movio linear y""")

        # yaw align
        yaw_initial = ang
        dir_ang = (-1 if yaw_initial - yaw_goal > 0 else 1)  # sign?
        if yaw_initial-yaw_goal != 0:
            rp.loginfo(f"""se alineo angular""")
            instructions.append((0, self.v_ang * dir_ang, abs(yaw_goal - yaw_initial) / self.v_ang))

        self.aplicar_velocidad(instructions)
        self.current_pose = goal_pose
        
    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            self.mover_robot_a_destino(pose)
            time.sleep(1)

if __name__ == "__main__":
    time.sleep(5)
    m = Movement()
    rp.spin()
