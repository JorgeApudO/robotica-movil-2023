#!/usr/bin/env python3
import rospy as rp
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


LOWER_ANGLE_LIMIT = -57
UPPER_ANGLE_LIMIT = 57


class Robot:
    states = [""]

    def __init__(self):
        # ---
        # Initial conditions
        self.__state = ""
        self.map_matrix = None
        self.map_data = None

        self.angle_increment = None
        self.lidar_scan = None

        # ---
        # Map loader
        rp.Subscriber("/map", OccupancyGrid, self.load_map_grid)

        # ---
        # Lidar data
        rp.Subscriber("/scan", LaserScan, self.update_laser)

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, new):
        if new in self.states:
            self.__state = new
        else:
            rp.logerr(f"<> State {new} not found")

    def load_map_grid(self, map):
        info = map.info
        width = int(info.width)
        height = int(info.height)

        data = map.data

        self.map_matrix = [[data[i*height + j] for j in range(width)] for i in range(height)]
        self.map_data = info

    def update_laser(self, data):
        angle_min = float(data.angle_min)
        angle_inc = float(data.angle_increment)

        ranges = np.array(data.ranges)

        ranges[ranges < float(data.range_min)] = np.NaN
        ranges[ranges > float(data.range_max)] = np.NaN

