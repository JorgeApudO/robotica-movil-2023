#!/usr/bin/env python3
import rospy
# import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time


class Robot():
    def __init__(self):
        rospy.init_node("follow_square_robot")

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.ang = 0.0
        self.goal_ang = 0.0

        self.vel = 0.2
        self.ang_vel = 0.0

        # --------------------------------------------------------------------
        # VEL PUBLISHER NODE
        # --------------------------------------------------------------------
        self.vel_applier = rospy.Publisher("yocs_cmd_vel_mux/input/navigation",
                                           Twist, queue_size=1)

        # --------------------------------------------------------------------
        # ANGLE PID CONTROL
        # --------------------------------------------------------------------
        self.ang_set_point = rospy.Publisher('/blue_square/setpoint',
                                             Float64, queue_size=1)
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_actuation = rospy.Subscriber('/blue_square/control_effort',
                                              Float64, self.ang_actuation_fn)

        # --------------------------------------------------------------------
        # TIMER
        # --------------------------------------------------------------------
        # self.prev_time = time.time()
        self.period = 0.1
        # rospy.Timer(rospy.Duration(self.period), self.publish_vel)

    def ang_actuation_fn(self, data):
        self.ang_vel = float(data.data)
        self.publish_vel()

    def publish_vel(self, data=None):
        # rospy.loginfo(f"{time.time() - self.prev_time}")
        vel = Twist()
        vel.linear.x = self.vel
        vel.angular.z = self.ang_vel * -1
        rospy.loginfo(f"VEL: {self.ang_vel}")
        self.vel_applier.publish(vel)
        # self.prev_time = time.time()


if __name__ == "__main__":
    robot = Robot()
    robot.ang_set_point.publish(0)
    rospy.spin()
