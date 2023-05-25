#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv
import time

class Robot():
    def __init__(self):
        rospy.init_node("reckoning_robot")
        # time.sleep(10)
        # --------------------------------------------------------------------
        # CONSTANTS
        # --------------------------------------------------------------------
        self.dist_threshold = 0.01
        self.ang_threshold = np.pi / 180

        self.min_front_dist = 0.75  # [m] Distancia a la que se detiene

        self.bridge = CvBridge()
        self.img_guardadas = 0

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.distance = np.array((0.0, 0.0, 0.80))
        self.ang = 0.0
        self.arrow_rotation = False
        self.get_direction = False
        self.vel = 0.1
        self.ang_vel = 0.0

        # --------------------------------------------------------------------
        # VEL PUBLISHER NODE
        # --------------------------------------------------------------------
        self.vel_applier = rospy.Publisher("yocs_cmd_vel_mux/input/navigation",
                                           Twist, queue_size=1)

        # --------------------------------------------------------------------
        # POSE NODE
        # --------------------------------------------------------------------

        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw',
                                         Image, self.process_depth, queue_size=1)
        self.depth_pub = rospy.Publisher('image_raw_2',Image, queue_size=2)
        
        self.img_pub = rospy.Publisher('image_filtered',Image, queue_size=2)

        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_color',
                                        Image, self.arrow_detector, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry,
                                         self.odom_fn)
        # --------------------------------------------------------------------
        # ANGLE P CONTROL
        # --------------------------------------------------------------------
        self.ang_set_point = rospy.Publisher('/angle/setpoint',
                                              Float64, queue_size=1)
        rospy.loginfo("Waiting angle distance pid setpoint process")
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_state = rospy.Publisher('/angle/state',
                                                   Float64, queue_size=1)
        rospy.loginfo("Waiting angle distance pid state process")
        while self.ang_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.ang_actuation = rospy.Subscriber('/angle/control_effort',
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

        if self.stopped():
            self.vel = 0
            rospy.loginfo('paro')
        else:
            self.vel = 0.1

        if self.contin() or self.arrow_rotation:
            self.ang_vel = float(data.data)
        else:
            self.ang_vel = 0
        self.check_rotate()
        # rospy.loginfo(f"Angular speed received: {self.ang_vel}")
        self.publish_vel()

    def process_depth(self, data: Image):
        # Procesar distancia en la imagen con numpy
        depth_image = self.bridge.imgmsg_to_cv2(data).copy()
        self.distance = np.array(get_image_means(self, depth_image))

    def publish_depth(self, data):
        # Publish difference between left and right distance
        if not self.arrow_rotation:
            dif_distance =  self.distance[1]-self.distance[0]
            self.ang_state.publish(dif_distance)

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z =  self.ang_vel
        self.vel_applier.publish(velocity)

    def stopped(self):
        # Podriamos añadirle caso en que ve la flecha suficientemente bien
        return self.distance[2] <= self.min_front_dist

    def contin(self):
        if self.arrow_rotation:
            return abs(self.distance[1]-self.distance[0]) > 10
        else:
            return False
    
    def check_rotate(self):
        if self.stopped() and not self.contin():
            self.arrow_rotation = True

    def arrow_detector(self, data):
        # Deteccion de la flecha
        if self.arrow_rotation and not self.get_direction:
            zeros = np.zeros(2)
            img = self.bridge.imgmsg_to_cv2(data)[100:300, :]
            red_filtered = get_red_mask(img) 
            gray = cv.cvtColor(red_filtered, cv.COLOR_BGR2GRAY)
            edges = cv.Canny(gray, 50, 150, apertureSize=3)
            lines_positions = cv.HoughLinesP(edges, 1, np.pi/180, 100,
                                             minLineLength=100,
                                             maxLineGap=10)[:, 0]
            
            rectas = np.apply_along_axis(pos_y_pendiente, 1, lines_positions)
            rectas = rectas[rectas != zeros]

            # No se si esto esta bien
            if np.mean(rectas) < img.shape[1]/2:
                self.goal_ang = sawtooth(np.pi/2 + self.ang)
            else:
                self.goal_ang = sawtooth(-np.pi/2 + self.ang)
            self.get_direction = True

        if self.get_direction:
            ang_diff = min_rotation_diff(self.goal_ang, self.ang)
            self.ang_state.publish(ang_diff)


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




def get_image_means(self,depth_image):
    # Left depth from image
    depth_left = depth_image[200:300, :40]
    # Right depth from image
    depth_right = depth_image[200:300, -40:]
    # Center depth from image
    depth_center = depth_image[200:300, 150:-150]
    new = np.concatenate((depth_left,np.concatenate((depth_center,depth_right),axis=1)),axis=1)
    new = self.bridge.cv2_to_imgmsg(new, encoding="passthrough")
    self.depth_pub.publish(new)
    prom_center = np.nanmean(depth_center)
    if np.isnan(prom_center):
        prom_center = 0.0
    prom_left = np.nanmean(depth_left)
    if np.isnan(prom_left):
        prom_left = 0.0
    prom_right = np.nanmean(depth_right)
    if np.isnan(prom_right):
        prom_right = 0.0
    rospy.loginfo(f"Current distances {prom_center},{prom_left},{prom_right}")
    return prom_left, prom_right, prom_center


def get_centers(mask):
    bilateral = cv.bilateralFilter(mask, 9, 75, 75)
    median = cv.medianBlur(bilateral, 7)
    M = cv.moments(median)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    else:
        return (-1, -1)


def get_red_mask(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    mask0 = cv.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
    mask1 = cv.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]))
    mask = mask0 + mask1

    return mask


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
