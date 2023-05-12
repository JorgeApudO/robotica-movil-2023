import rospy
from std_msgs import Float64, PoseArray, Twist, Pose
from tf.transformations import euler_from_quaternion
import numpy as np


def sawtooth(rad):
    return (rad - np.pi) % (2*np.pi) - np.pi


class Robot():
    def __init__(self):
        rospy.init_node("reckoning_robot")

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.pos = np.array((0.0, 0.0))
        self.goal_pos = np.array((0.0, 0.0))
        self.ang = 0.0
        self.goal_ang = 0.0

        self.vel = 0.0
        self.ang_vel = 0.0

        # --------------------------------------------------------------------
        # VEL PUBLISHER NODE
        # --------------------------------------------------------------------
        self.vel_applier = rospy.Publisher("yocs_cmd_vel_mux/input/navigation",
                                           Twist, queue_size=1)

        # --------------------------------------------------------------------
        # REAL POSE NODE
        # --------------------------------------------------------------------
        self.real_pose_sub = rospy.Subscriber('real_pose', Pose,
                                              self.real_pose_fn)

        # --------------------------------------------------------------------
        # DISTANCE PID CONTROL
        # --------------------------------------------------------------------
        self.dist_set_point = rospy.Publisher('/reckoning_dist/setpoint',
                                              Float64, queue_size=1)
        while self.dist_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.dist_state = rospy.Publisher('/reckoning_dist/state',
                                          Float64, queue_size=1)
        while self.dist_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.dist_actuation = rospy.Subscriber('/reckoning_dist/control_effort',
                                               Float64, self.dist_actuation_fn)

        # --------------------------------------------------------------------
        # ANGLE PID CONTROL
        # --------------------------------------------------------------------
        self.ang_set_point = rospy.Publisher('/reckoning_ang/setpoint',
                                             Float64, queue_size=1)
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_state = rospy.Publisher('/reckoning_ang/state',
                                         Float64, queue_size=1)
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
            raw_ang = euler_from_quaternion((pose.orientation.x,
                                             pose.orientation.y,
                                             pose.orientation.z,
                                             pose.orientation.w))[2]
            self.goal_ang = sawtooth(raw_ang)
            # Girar
            self.ang_set_point.publish(0)
            # Esperar a que este alineado con goal
            while abs(self.goal_ang - self.ang) > self.ang_threshold:
                rospy.sleep(self.period)
            # Moverse hasta estar cerca
            self.dist_set_point.publish(0)
            while abs(self.goal_pos - self.pos) > self.dist_threshold:
                rospy.sleep(self.period)

    def ang_actuation_fn(self, data):
        self.ang_vel = float(data.data)
        rospy.loginfo(f"Angular speed received: {self.ang_vel}")
        self.publish_vel()

    def dist_actuation_fn(self, data):
        self.vel = float(data.data)
        rospy.loginfo(f"Speed received: {self.vel}")
        self.publish_vel()

    def publish_odom(self):
        # Publish position and angle to self.dist_state and self.ang_state
        goal_vec = self.goal_pos - self.pos
        self.dist_state.publish(np.linalg.norm(goal_vec))
        self.ang_state.publish(np.arctan2(goal_vec[1], goal_vec[0]) - self.ang)

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
