#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np


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

        # --------------------------------------------------------------------
        # CONSTANTS
        # --------------------------------------------------------------------
        self.dist_threshold = 0.01
        self.ang_threshold = np.pi / 180

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.pos = np.array((0.0, 0.0))
        self.goal_pos = np.array((0.0, 0.0))
        self.ang = 0.0
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

    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            self.goal_pos = np.array((pose.position.x, pose.position.y))
            goal_vec = self.goal_pos - self.pos
            ang_diff = (np.arctan2(goal_vec[1], goal_vec[0]) - self.ang)

            # Girar
            # Esperar a que este alineado con goal
            self.ang_set_point.publish(0)
            rospy.loginfo("Waiting for alignment")
            self.rotating = True
            while abs(ang_diff) > self.ang_threshold:
                rospy.sleep(self.period)
                ang_diff = (np.arctan2(goal_vec[1], goal_vec[0]) - self.ang)
            self.rotating = False

            # Moverse hasta estar cerca
            self.dist_set_point.publish(0)
            rospy.loginfo("Start movement")
            while np.linalg.norm(goal_vec) > self.dist_threshold:
                rospy.sleep(self.period)
                goal_vec = self.goal_pos - self.pos

        self.stop = True

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

    def odom_fn(self, data: Odometry):
        pose_c = data.pose
        pose = pose_c.pose

        pos = pose.position
        orient = pose.orientation

        self.pos = np.array((pos.x, pos.y))

        raw_ang = euler_from_quaternion((orient.x, orient.y,
                                         orient.z, orient.w))[2]
        self.ang = sawtooth(raw_ang)

        rospy.loginfo(f"POSITION: {self.pos}")

    def publish_odom(self, data):
        # Publish position and angle to self.dist_state and self.ang_state
        goal_vec = self.goal_pos - self.pos
        self.dist_state.publish(np.linalg.norm(goal_vec))
        goal_ang = np.arctan2(goal_vec[1], goal_vec[0])

        ang_diff = min_rotation_diff(goal_ang, self.ang)

        # rospy.loginfo(f"POSITION: {self.pos} GOAL: {self.goal_pos} |  ANGLE DIFF: {ang_diff}  |  VEL: {self.vel}")
        self.ang_state.publish(ang_diff)

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

    def real_pose_fn(self, data):
        pos = data.position
        orient = data.orientation

        self.pos = np.array((pos.x, pos.y))

        raw_ang = euler_from_quaternion((orient.x, orient.y,
                                         orient.z, orient.w))[2]
        self.ang = sawtooth(raw_ang)


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
