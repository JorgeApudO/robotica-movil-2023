#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import os


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


class Robot():
    def __init__(self):
        rospy.init_node("reckoning_robot")
        rospy.loginfo(os.getcwd())

        # --------------------------------------------------------------------
        # CONSTANTS
        # --------------------------------------------------------------------
        self.dist_threshold = 0.001
        self.ang_threshold = np.pi / 180 / 2

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.pos = np.array((0.0, 0.0))
        self.real_pos = np.array((0.0, 0.0))
        self.goal_pos = np.array((0.0, 0.0))
        self.ang = 0.0
        self.real_ang = 0.0
        self.goal_ang = 0.0

        self.vel = 0.0
        self.ang_vel = 0.0

        self.rotating = False
        self.stop = False

        # --------------------------------------------------------------------
        # VEL PUBLISHER NODE
        # --------------------------------------------------------------------
        self.vel_applier = rospy.Publisher("yocs_cmd_vel_mux/input/navigation",
                                           Twist, queue_size=1)

        # --------------------------------------------------------------------
        # POSE NODE
        # --------------------------------------------------------------------
        # self.real_pose_sub = rospy.Subscriber('real_pose', Pose,
        #                                       self.real_pose_fn)

        self.odom_sub = rospy.Subscriber('odom', Odometry,
                                         self.odom_fn)

        self.real_sub = rospy.Subscriber('real_pose', Pose,
                                         self.real_pose_fn)

        # --------------------------------------------------------------------
        # DISTANCE PID CONTROL
        # --------------------------------------------------------------------
        self.dist_set_point = rospy.Publisher('/reckoning_dist/setpoint',
                                              Float64, queue_size=1)
        rospy.loginfo("Waiting dist pid setpoint process")
        while self.dist_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.dist_state = rospy.Publisher('/reckoning_dist/state',
                                          Float64, queue_size=1)
        rospy.loginfo("Waiting dist pid state process")
        while self.dist_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.dist_actuation = rospy.Subscriber('/reckoning_dist/control_effort',
                                               Float64, self.dist_actuation_fn)

        # --------------------------------------------------------------------
        # ANGLE PID CONTROL
        # --------------------------------------------------------------------
        self.ang_set_point = rospy.Publisher('/reckoning_ang/setpoint',
                                             Float64, queue_size=1)
        rospy.loginfo("Waiting angle pid setpoint process")
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_state = rospy.Publisher('/reckoning_ang/state',
                                         Float64, queue_size=1)
        rospy.loginfo("Waiting ang pid state process")
        while self.ang_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_actuation = rospy.Subscriber('/reckoning_ang/control_effort',
                                              Float64, self.ang_actuation_fn)

        # --------------------------------------------------------------------
        # TIMER
        # --------------------------------------------------------------------
        self.period = 0.1
        rospy.Timer(rospy.Duration(self.period), self.publish_odom)

        # --------------------------------------------------------------------
        # POSE_LOADER
        # --------------------------------------------------------------------
        self.mov_sub = rospy.Subscriber("goal_list", PoseArray,
                                        self.accion_mover)

        self.data = [["time", "pos_x", "pos_y", "ang_ref", "pos_ref", "ang_act", "pos_act", "ang_out", "pos_out"]]

    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            # for key in self.datos.keys():
            #     self.datos[key].append([])

            goal_pos = np.array((pose.position.x, pose.position.y))
            goal_vec = goal_pos - self.pos

            self.goal_ang = np.arctan2(goal_vec[1], goal_vec[0])
            ang_diff = self.goal_ang - self.ang

            # Girar
            # Esperar a que este alineado con goal
            self.ang_set_point.publish(0)
            rospy.loginfo("Waiting for alignment")
            self.rotating = True
            while abs(ang_diff) > self.ang_threshold:
                rospy.sleep(self.period)
                ang_diff = self.goal_ang - self.ang
            self.rotating = False

            self.goal_pos = goal_pos
            # Moverse hasta estar cerca
            self.dist_set_point.publish(0)
            rospy.loginfo("Start movement")
            while np.linalg.norm(goal_vec) > self.dist_threshold:
                rospy.sleep(self.period)
                goal_vec = self.goal_pos - self.pos

        self.stop = True
        dir_path = os.path.dirname(os.path.realpath(__file__))

        with open(os.path.join(dir_path, "data.csv"), 'w') as file:
            file.writelines([','.join((str(el) for el in line))+'\n' for line in self.data])

        rospy.loginfo(f"Final pos: {self.real_pos}")
        rospy.loginfo(f"Final error: {np.linalg.norm(self.real_pos)}")
        rospy.signal_shutdown("finished")

    def ang_actuation_fn(self, data: Float64):
        if self.stop:
            self.ang_vel = 0
        else:
            self.ang_vel = float(data.data)
        # rospy.loginfo(f"Angular speed received: {self.ang_vel}")
        self.publish_vel()

    def dist_actuation_fn(self, data: Float64):
        if self.rotating or self.stop:
            self.vel = 0
        else:
            self.vel = float(data.data) * -1
        # rospy.loginfo(f"Speed received: {self.vel}")
        self.publish_vel()

    def log_data(self):
        t = time.time()
        pos_x = self.real_pos[0] - 1.0
        pos_y = self.real_pos[1] - 1.0

        ang_ref = 0
        ang_act = self.ang_vel
        ang_out = min_rotation_diff(self.goal_ang, self.real_ang)

        pos_ref = 0
        pos_act = self.vel
        pos_out = np.linalg.norm(self.goal_pos - (self.real_pos - np.array([1.0, 1.0])))

        self.data.append([t, pos_x, pos_y, ang_ref, pos_ref,
                          ang_act, pos_act, ang_out, pos_out])

    def odom_fn(self, data: Odometry):
        pose_c = data.pose
        pose = pose_c.pose

        pos = pose.position
        orient = pose.orientation

        self.pos = np.array((pos.x, pos.y))

        raw_ang = euler_from_quaternion((orient.x, orient.y,
                                         orient.z, orient.w))[2]
        self.ang = sawtooth(raw_ang)

        rospy.loginfo(f"DISTANCE: {np.linalg.norm(self.goal_pos - (self.real_pos - np.array([1.0, 1.0])))}")

    def publish_odom(self, data):
        # Publish position and angle to self.dist_state and self.ang_state
        goal_vec = self.goal_pos - self.pos
        self.dist_state.publish(np.linalg.norm(goal_vec))

        if not self.rotating:
            self.goal_ang = np.arctan2(goal_vec[1], goal_vec[0])

        ang_diff = min_rotation_diff(self.goal_ang, self.ang)

        # rospy.loginfo(f"POSITION: {self.pos} GOAL: {self.goal_pos} |  ANGLE DIFF: {ang_diff}  |  VEL: {self.vel}")
        self.ang_state.publish(ang_diff)

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

        self.log_data()

    def real_pose_fn(self, data):
        pos = data.position
        orient = data.orientation

        self.real_pos = np.array((pos.x, pos.y))

        raw_ang = euler_from_quaternion((orient.x, orient.y,
                                         orient.z, orient.w))[2]
        self.real_ang = sawtooth(raw_ang)


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
