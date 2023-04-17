#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray,Pose,Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion

from os import path
import math


def parse_value(value) -> float:
    tokens = value.split('/')

    sign = -1 if '-' in tokens[0] else 1

    ponderator = math.pi if 'pi' in tokens[0] else 1

    numerator = tokens[0].replace('-', '').replace('pi', '')

    factor = int(numerator) if numerator != '' else 1

    divisor = 1 if len(tokens) == 1 else int(tokens[1])

    return sign * factor * ponderator / divisor    


def pose_loader():
    rospy.init_node( 'pose_loader' )
    pub = rospy.Publisher( 'goal_list', PoseArray, queue_size=10 )
    with open("/home/jorge/robotica-movil-2023/src/lab1/scripts/pose_list.txt", "r") as pl:
        pose_list = [line.strip()[1:-1].split(",") for line in pl]
        parray = PoseArray()
        for p in pose_list:
            pose = Pose()
            y = parse_value(p[2])
            q = quaternion_from_euler(0, 0, y)
            q =Quaternion(*q)
            pose.orientation = q
            pose.position = Point(float(p[0]),float(p[1]),0)
            parray.poses.append(pose)
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        pass
    pub.publish(parray)

if __name__ == '__main__':
    pose_loader()
    rospy.spin()