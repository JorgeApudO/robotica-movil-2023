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
        self.mov_sub = rp.Subscriber("goal_list", PoseArray, self.accion_mover)  # investigar PoseArray
        self.current_pose = Pose()

    def aplicar_velocidad(self, speed_command: list):
        for mov in speed_command:
            inicial_time = time.time()
            while time.time() <= inicial_time+mov[2]:
                speed = Twist()
                speed.linear.x, speed.angular.z = mov[0], mov[1]
                rp.loginfo("linear speed:", mov[0], "angular speed:", mov[1])
                self.vel_applier.publish(speed)
                self.rate.sleep()

    def mover_robot_a_destino(self, goal_pose: Pose):  # EDITAR POSEARRAY
        x_goal, y_goal = goal_pose.position.x(), goal_pose.position.y()
        yaw_goal = euler_from_quaternion(*goal_pose.orientation())[2]
        instructions = []  # Hay que definir esto bien
        x_initial,y_initial = self.current_pose.position.x(), self.current_pose.position.y()
        yaw_initial = euler_from_quaternion(*self.current_pose.orientation())[2]
        #align x
        ang = (0 if x_goal - x_initial >= 0 else math.pi)
        dir_ang = (-1 if yaw_initial - ang > 0 else 1)
        instructions.append((0, self.v_ang * dir_ang, abs(ang - yaw_initial) / self.v_ang))
        instructions.append((self.v, 0, abs(x_initial - x_goal) / self.v))

        # align y
        yaw_initial = ang
        ang = (math.pi/2 if y_goal - y_initial > 0 else -math.pi/2)
        dir_ang = (-1 if yaw_initial - ang > 0 else 1) #sign?
        
        instructions.append((0, self.v_ang * dir_ang, abs(ang - yaw_initial) / self.v_ang))
        instructions.append((self.v, 0, abs(y_initial - y_goal) / self.v))
        
        #yaw align
        yaw_initial = ang
        dir_ang = (-1 if yaw_goal - yaw_initial > 0 else 1) #sign?
        instructions.append((0, self.v_ang * dir_ang, abs(yaw_goal - yaw_initial) / self.v_ang))
        
        self.aplicar_velocidad(instructions)
        self.current_pose = goal_pose

    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            self.mover_robot_a_destino(pose)
