#!/usr/bin/env python3
import rospy as rp
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Int8, Bool
from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler


LOWER_ANGLE_LIMIT = -27 * np.pi / 180
UPPER_ANGLE_LIMIT = 27 * np.pi / 180


PARTICLE = 1 # sensor model and particle 
WAIT = 2 # procesando info
MOVEMENT = 3 # move robot
DONE = 4 #Finish

class RobotBrain:
    states = {PARTICLE, WAIT, MOVEMENT, DONE}

    def __init__(self):
        rp.init_node("Robot_Brain")
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
        # State updater
        rp.Subscriber("/task_done", Int8, self.update_state)
        # ---
        # Map loader
        rp.Subscriber("/map", OccupancyGrid, self.load_map_grid)

        # ---
        # Lidar data
        rp.Subscriber("/scan", LaserScan, self.update_laser)

        # ---
        # Movement manager
        self.movement_pub = rp.Publisher(
            "/enable_movement", Bool, queue_size=1)
        # ---
        # Sensor data publisher
        self.measurement_pub = rp.Publisher('/distances', Float64MultiArray,
                                            queue_size=2)
        self.movement_depth_pub = rp.Publisher('/depth_data', Float64MultiArray,
                                            queue_size=2)
        self.period = 0.1
        rp.Timer(rp.Duration(self.period), self.update)

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

    def update(self, *args):
        if self.state == PARTICLE:
            self.publish_lidar()
            self.state = WAIT
            self.movement_pub.publish(False)
        elif self.state == MOVEMENT:
            self.publish_lidar()
            self.movement_pub.publish(True)
        elif self.state == DONE:
            self.movement_pub.publish(False)
            self.notify()

    def update_laser(self, data):
        angle_min = float(data.angle_min)
        angle_max = float(data.angle_max)
        self.angle_inc = float(data.angle_increment)

        ranges = data.ranges

        index_min = int(
            np.ceil((LOWER_ANGLE_LIMIT - angle_min) / self.angle_inc))
        index_max = len(
            ranges) - int(np.ceil((angle_max - UPPER_ANGLE_LIMIT) / self.angle_inc))

        ranges = np.array(ranges[index_min:index_max+1])

        ranges[ranges < float(data.range_min)] = np.NaN
        ranges[ranges > float(data.range_max)] = np.NaN

        self.lidar_scan = ranges

        # rp.loginfo(ranges)

    def update_state(self, data):
        self.state = data.data

    def publish_lidar(self):
        lidar_info = np.array((x, LOWER_ANGLE_LIMIT + i*self.angle_inc)
                              for i, x in enumerate(self.lidar_scan))
        if self.state == MOVEMENT:
            self.movement_depth_pub.publish(lidar_info)

        elif self.state == PARTICLE:
            self.measurement_pub.publish(lidar_info)
            

    def notify(self):
        # Avisar por los parlantes y todo eso
        pass

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
    robot = RobotBrain()
    rp.spin()
