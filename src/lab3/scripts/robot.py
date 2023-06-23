#!/usr/bin/env python3
import rospy as rp
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler


LOWER_ANGLE_LIMIT = -27 * np.pi / 180
UPPER_ANGLE_LIMIT = 27 * np.pi / 180


class Robot:
    states = [""]

    def __init__(self):
        rp.init_node("localization_robot")
        pub_initial_pose(0.5, 0.5, 0)

        self.flip = bool(rp.get_param('/localization/flip_map', 0))
        rp.loginfo(self.flip)
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

        if self.flip:
            map_matrix = np.flip(map_matrix, axis=0)

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

        # rp.loginfo(ranges)


def pub_initial_pose(x, y, yaw):
    pub_init_pose = rp.Publisher('initial_pose', Pose, queue_size=1)

    while pub_init_pose.get_num_connections() == 0 and not rp.is_shutdown():
        rp.sleep(0.2)

    init_pose = Pose()
    init_pose.position.x = x
    init_pose.position.y = y

    x, y, z, w = quaternion_from_euler(0, 0, yaw)

    init_pose.orientation.x = x
    init_pose.orientation.y = y
    init_pose.orientation.z = z
    init_pose.orientation.w = w

    pub_init_pose.publish(init_pose)


if __name__ == "__main__":
    robot = Robot()
    rp.spin()
