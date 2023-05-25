#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv
from blue_square_detector import get_centers, get_mask


def sawtooth(rad):
    return (rad - np.pi) % (2*np.pi) - np.pi


def min_rotation_diff(goal, actual):
    if abs(goal - actual) > np.pi:
        if actual < 0:
            return 2*np.pi + actual - goal
        else:
            return actual - goal - 2*np.pi
    else:
        return actual - goal


def pos_y_pendiente(pos):
    x1, y1, x2, y2 = pos
    if x2-x1 == 0 or y2-y1 == 0:
        return np.zeros(2)
    return np.array(x1, x2)


def filter_color(rgb_img, filter_hue):
    hsv = cv.cvtColor(rgb_img, cv.COLOR_BGR2HSV)

    if filter_hue - 10 < 0:
        lower = 180 + filter_hue - 10
    else:
        lower = filter_hue - 10

    if filter_hue + 10 > 180:
        upper = filter_hue + 10 - 180
    else:
        upper = filter_hue + 10

    lower_limit = np.array([lower, 100, 50])
    upper_limit = np.array([upper, 255, 255])

    mask = cv.inRange(hsv, lower_limit, upper_limit)

    return mask


class Robot():
    def __init__(self):
        rospy.init_node("reckoning_robot")

        # --------------------------------------------------------------------
        # CONSTANTS
        # --------------------------------------------------------------------
        self.dist_threshold = 0.01
        self.ang_threshold = np.pi / 180

        self.min_front_dist = 650  # [m] Distancia a la que se detiene

        self.bridge = CvBridge()
        self.img_guardadas = 0

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.distance = np.array((0.0, 0.0, 0.0))
        self.ang = 0.0
        self.arrow_rotation = False
        self.vel = 0.02
        self.ang_vel = 0.0

        # --------------------------------------------------------------------
        # VEL PUBLISHER NODE
        # --------------------------------------------------------------------
        self.vel_applier = rospy.Publisher("yocs_cmd_vel_mux/input/navigation",
                                           Twist, queue_size=1)

        # --------------------------------------------------------------------
        # POSE NODE
        # --------------------------------------------------------------------

        self.deph_sub = rospy.Subscriber('/camera/depth/image_raw',
                                         Image, self.process_depth, queue_size=1)

        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_color',
                                        Image, self.arrow_detector, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry,
                                         self.odom_fn)
        # --------------------------------------------------------------------
        # WALL DISTANCE P CONTROL
        # --------------------------------------------------------------------
        self.wall_set_point = rospy.Publisher('/wall_distance/setpoint',
                                              Float64, queue_size=1)
        rospy.loginfo("Waiting wall distance pid setpoint process")
        while self.wall_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.wall_set_point.publish(0)

        self.wall_distance_state = rospy.Publisher('/wall_distance/state',
                                                   Float64, queue_size=1)
        rospy.loginfo("Waiting wall distance pid state process")
        while self.wall_distance_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_actuation = rospy.Subscriber('/wall_distance/control_effort',
                                              Float64, self.ang_actuation_fn)

        # --------------------------------------------------------------------
        # ANGLE PID CONTROL
        # --------------------------------------------------------------------
        self.ang_set_point = rospy.Publisher('/reckoning_ang/setpoint',
                                             Float64, queue_size=1)
        rospy.loginfo("Waiting angle pid setpoint process")
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_state = rospy.Publisher('/reckoning_ang/state',
                                         Float64, queue_size=1)
        rospy.loginfo("Waiting ang pid state process")
        while self.ang_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_actuation = rospy.Subscriber('/reckoning_ang/control_effort',
                                              Float64, self.ang_actuation_fn)
        self.ang_set_point.publish(0)

        # --------------------------------------------------------------------
        # TIMER
        # --------------------------------------------------------------------
        self.period = 0.1
        rospy.Timer(rospy.Duration(self.period), self.publish_depth)

    def odom_fn(self, data: Odometry):
        pose_c = data.pose
        pose = pose_c.pose
        orient = pose.orientation

        raw_ang = euler_from_quaternion((orient.x, orient.y,
                                         orient.z, orient.w))[2]
        self.ang = sawtooth(raw_ang)

    def ang_actuation_fn(self, data: Float64):

        if self.stopped() and not self.arrow_rotation:
            self.vel = 0
            self.ang_vel = 0
        elif self.stopped() and self.arrow_rotation:
            self.vel = 0
            self.ang_vel = float(data.data)
        else:
            self.vel = 0.02
            self.ang_vel = float(data.data)

        # rospy.loginfo(f"Angular speed received: {self.ang_vel}")
        self.publish_vel()

    def process_depth(self, data: Image):
        # Procesar distancia en la imagen con numpy
        depth_image = self.bridge.imgmsg_to_cv2(data).copy()
        """
        Guardar imagenes para poder medir correctamente los pixeles pa cortar

        if self.img_guardadas<10:
            cv2.imwrite(f"imagen_prof{self.img_guardadas}.png", depth_image)
            print("imagen guardada")
        """

        # Left depth from image
        depth_left = depth_image[100:300, :30]
        depth_left = depth_left[depth_left <= 1000]
        # Right depth from image
        depth_right = depth_image[100:300, -30:]
        depth_right = depth_right[depth_right <= 1000]
        # Center depth from image
        depth_center = depth_image[100:300, 150:-150]
        depth_center = depth_center[depth_center <= 1000]

        prom_center = float(np.nanmean(depth_center))
        prom_left = float(np.nanmean(depth_left))
        prom_right = float(np.nanmean(depth_right))

        self.distance = np.array(
            (prom_left, prom_right, prom_center))

    def publish_depth(self, data):
        # Publish difference between left and right distance
        dif_distance = self.distance[0] - self.distance[1]
        self.wall_distance_state.publish(dif_distance)

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

    def stopped(self):
        # Podriamos añadirle caso en que ve la flecha suficientemente bien
        return self.distance[2] <= self.min_front_dist

    def arrow_detector(self, data):
        zeros = np.zeros(2)

        # Deteccion de la flecha
        if self.stopped():

            self.arrow_rotation = True
            # el bridge es unico?
            img = self.bridge.imgmsg_to_cv2(data)[100:300, :]
            red_filtered = filter_color(img, 0)
            # ver si es necesario centrar el robot primero para luego girar
            gray = cv.cvtColor(red_filtered, cv.COLOR_BGR2GRAY)
            edges = cv.Canny(gray, 50, 150, apertureSize=3)
            lines_positions = cv.HoughLinesP(edges, 1, np.pi/180, 100,
                                             minLineLength=100,
                                             maxLineGap=10)[:, 0]

            rectas = np.apply_along_axis(pos_y_pendiente, 1, lines_positions)
            rectas = rectas[rectas != zeros]

            # No se si esto esta bien
            if np.mean(rectas) < img.shape[1]/2:
                goal_ang = np.pi/2
            else:
                goal_ang = (-np.pi/2)

            ang_diff = min_rotation_diff(goal_ang, self.ang)
            self.ang_state.publish(ang_diff)


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
