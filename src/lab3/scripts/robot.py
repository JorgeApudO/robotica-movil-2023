#!/usr/bin/env python3
import rospy as rp
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


LOWER_ANGLE_LIMIT = -27 * np.pi / 180
UPPER_ANGLE_LIMIT = 27 * np.pi / 180


class Robot:
    states = [""]

    def __init__(self):
        rp.init_node("localization_robot")
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

        map_matrix = 100 - np.array(map.data).reshape((height, width))
        map_matrix = (map_matrix * (255/100.0)).astype(np.uint8)

        self.map_matrix = map_matrix
        self.resolution = info.resolution

        rp.loginfo(self.map_matrix)
        rp.loginfo(self.map_data)

    def update_laser(self, data):
        angle_min = float(data.angle_min)
        angle_max = float(data.angle_max)
        angle_inc = float(data.angle_increment)

        ranges = data.ranges

        index_min = int(np.ceil((LOWER_ANGLE_LIMIT - angle_min) / angle_inc))
        index_max = len(
            ranges) - int(np.ceil((angle_max - UPPER_ANGLE_LIMIT) / angle_inc))

        '''
        rp.loginfo(f"\nlimits: {LOWER_ANGLE_LIMIT}, {UPPER_ANGLE_LIMIT}\n" +
                   f"angle_min_max: {angle_min}, {angle_max}\n" +
                   f"indexes: {index_min}, {index_max}\n" +
                   f"inc*index: {index_min*angle_inc}, {index_max*angle_inc}\n" +
                   f"len: {len(ranges)}")
        '''

        ranges = np.array(ranges[index_min:index_max+1])

        ranges[ranges < float(data.range_min)] = np.NaN
        ranges[ranges > float(data.range_max)] = np.NaN

        self.lidar_scan = ranges

        rp.loginfo(ranges)


if __name__ == "__main__":
    robot = Robot()
    rp.spin()
