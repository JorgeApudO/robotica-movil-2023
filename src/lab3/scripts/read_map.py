#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2


def set_map(occupancy_grid):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    map_resolution = occupancy_grid.info.resolution
    rospy.loginfo('map resolution: (%d,%d)' % (height, width))
    mapimg = 100 - np.array(occupancy_grid.data).reshape((height, width))
    mapimg = (mapimg * (255/100.0)).astype(np.uint8)
    mapimg = cv2.cvtColor(mapimg, cv2.COLOR_GRAY2RGB)
    mapimg = np.flip(mapimg, axis=0)
    cv2.imshow('2D Map', mapimg)
    cv2.waitKey(1000)


if __name__ == '__main__':
    rospy.init_node('map_reader')
    rospy.Subscriber('/map', OccupancyGrid, set_map)
    rospy.spin()
