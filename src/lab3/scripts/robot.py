#!/usr/bin/env python3
import rospy as rp
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

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

        # ---
        # Sensor data publisher
        self.period = 0.1
        self.measurement_pub = rp.Publisher('distances', Float64MultiArray,
                                    queue_size = 2)
        rp.Timer(rp.Duration(self.period), self.publish_lidar)

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

        self.map_matrix = np.array([[data[i*height + j]
                                    for j in range(width)] for i in range(height)])
        self.map_data = info

        rp.loginfo(self.map_matrix)
        rp.loginfo(self.map_data)

    def update_laser(self, data):
        angle_min = float(data.angle_min)
        angle_max = float(data.angle_max)
        self.angle_inc = float(data.angle_increment)

        ranges = data.ranges

        index_min = int(np.ceil((LOWER_ANGLE_LIMIT - angle_min) / self.angle_inc))
        index_max = len(
            ranges) - int(np.ceil((angle_max - UPPER_ANGLE_LIMIT) / self.angle_inc))

        ranges = np.array(ranges[index_min:index_max+1])

        ranges[ranges < float(data.range_min)] = np.NaN
        ranges[ranges > float(data.range_max)] = np.NaN

        self.lidar_scan = ranges

        rp.loginfo(ranges)

    def publish_lidar(self, data):

        lidar_info = np.array((x, LOWER_ANGLE_LIMIT + i*self.angle_inc
                               ) for i, x in enumerate(self.lidar_scan))
        
        self.measurement_pub.publish(lidar_info)
        

if __name__ == "__main__":
    robot = Robot()
    rp.spin()
