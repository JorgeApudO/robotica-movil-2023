#!/usr/bin/env python3
import cv2 as cv
import rospy
import numpy as np
import sys
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int64


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


def get_mask(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_limit = np.array([100, 100, 50])
    upper_limit = np.array([110, 255, 255])

    mask = cv.inRange(hsv, lower_limit, upper_limit)

    return mask


class Node:
    def __init__(self):
        rospy.init_node('blue_square_detector')

        self.bridge = CvBridge()

        # --------------------------------------------------------------------
        # KINECT IMAGES NODE
        # --------------------------------------------------------------------
        self.kinect_sub = rospy.Subscriber('/camera/rgb/image_color',
                                           Image, self.process_image)

        # --------------------------------------------------------------------
        # DISTANCE PUBLISHER
        # --------------------------------------------------------------------
        self.distance_pub = rospy.Publisher('/blue_square/state',
                                            Int64, queue_size=1)
        while self.distance_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def process_image(self, data: Image):
        cv_image = self.bridge.imgmsg_to_cv2(data)
        _, width, _ = cv_image.shape

        img_mask = get_mask(cv_image)
        cx, cy = get_centers(img_mask)

        if cx < 0:
            self.publish_dist(0)
        else:
            self.publish_dist(width//2 - cx)

    def publish_dist(self, dx: int):
        self.distance_pub.publish(dx)


class Opt:
    def __init__(self, img):
        self.frame = img

    def mouse_click(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            b = self.frame[y, x, 0]
            g = self.frame[y, x, 1]
            r = self.frame[y, x, 2]
            print(f"R: {r} | G: {g} | B: {b}")


if __name__ == "__main__":
    initial_time = time.time()
    img = cv.imread("imgs/frame000100.png")

    if img is None:
        sys.exit("Could not read image")

    opts = Opt(img)
    cv.namedWindow("result")
    cv.setMouseCallback("result", opts.mouse_click)

    result = get_mask(img)

    cx, cy = get_centers(result)
    print(time.time() - initial_time)

    cv.circle(result, (cx, cy), 5, (0, 0, 0), -1)

    cv.imshow("result", result)
    # cv.imshow("mask", mask)
    k = cv.waitKey(0)

