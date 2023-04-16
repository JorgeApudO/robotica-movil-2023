#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray,Pose,Point
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

from os import path
import math


def pose_loader():
    rospy.init_node( 'pose_loader' )
    pub = rospy.Publisher( 'goal_list', PoseArray, queue_size=10 )
    with open("/home/jorge/robotica-movil-2023/src/lab1/scripts/pose_list.txt", "r") as pl:
        pose_list = [line.strip()[1:-1].split(",") for line in pl]
        parray = PoseArray()
        for p in pose_list:
            pose = Pose()
            if p[2].isdigit():
                y = float(p[2])%(2*math.pi)
            q = quaternion_from_euler(0, 0, y)
            q =Quaternion(*q)
            pose.orientation = q
            pose.position = Point(float(p[0]),float(p[1]),0)
            parray.poses.append(pose)
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        pass
    pub.publish(parray)
    rospy.loginfo(parray)

if __name__ == '__main__':
    pose_loader()
    rospy.spin()