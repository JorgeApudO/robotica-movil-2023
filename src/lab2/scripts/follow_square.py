#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class Robot():
    def __init__(self):
        rospy.init_node("follow_square_robot")

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
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
        # POSE NODE
        # --------------------------------------------------------------------
        self.odom_sub = rospy.Subscriber('odom', Odometry,
                                         self.odom_fn)

        # --------------------------------------------------------------------
        # ANGLE PID CONTROL
        # --------------------------------------------------------------------
        self.ang_set_point = rospy.Publisher('/blue_square/setpoint',
                                             Float64, queue_size=1)
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_actuation = rospy.Subscriber('/blue_square/control_effect',
                                              Float64, self.ang_actuation_fn)

        # --------------------------------------------------------------------
        # TIMER
        # --------------------------------------------------------------------
        self.period = 0.1

    def ang_actuation_fn(self, data):
        self.ang_vel = float(data.data)
        self.publish_vel()

    def odom_fn(self, data):
        pass

    def publish_vel(self):
        vel = Twist()
        vel.linear.x = self.vel
        vel.angular.z = self.ang_vel
        self.vel_applier.publish(vel)
