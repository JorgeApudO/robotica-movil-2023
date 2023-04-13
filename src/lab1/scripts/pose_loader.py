#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from os import path
import math
def pose_loader():
    rospy.init_node( 'pose_loader' )
    pub = rospy.Publisher( 'goal_list', PoseArray, queue_size=10 )
    with open(path.join("src","lab1","scripts","pose_list.txt"), "r") as pl:
        pose_list = [line.strip()[1:-1].split(",") for line in pl]
        for j,pose in enumerate(pose_list):
            for i,el in enumerate(pose):
                if "pi" in el:
                    pose_list[j][i]= math.pi
                    if len(el)>2: pose_list[j][i] /= int(el[3:])
                else:
                    pose_list[j][i] = float(el)
            pose_list[j]= tuple(pose_list[j])
    print(pose_list)
    pub.publish(pose_list)

if __name__ == '__main__':
    pose_loader()
    rospy.spin()