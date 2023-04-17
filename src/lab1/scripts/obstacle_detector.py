#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import numpy as np


class Turtlebot_Kinect(object):
    def __init__(self) -> None:
        self.raw_sub = rospy.Subscriber('/camera/depth/image_raw', Image,
                                        self.raw_cb)
        self.bridge = CvBridge()
        self.current_cv_raw_image = None

    def raw_cb(self, data: Image) -> None:
        self.current_cv_raw_image = self.bridge.imgmsg_to_cv2(data)


class Turtlebot_Perception(object):
    def __init__(self) -> None:
        self.pub = rospy.Publisher('/occupancy_state', Vector3, queue_size=10)
        self.rate_obj = rospy.Rate(5)
        self.kinect_obj = Turtlebot_Kinect()

    def send(self) -> None:
        while not rospy.is_shutdown():
            if self.kinect_obj.current_cv_raw_image is None:
                detected_objects = Vector3(0, 0, 0)
            else:
                detected_objects = self.detect_objects()
            self.pub.publish(detected_objects)
            self.rate_obj.sleep()

    def detect_objects(self) -> Vector3:
        raw_image = self.kinect_obj.current_cv_raw_image.copy()

        # revisar pixeles 0-213, 213-427, 427-640
        # cortar bordes abajo y arriba de matriz
        # revisar si hay objetos entre 450mm y 800mm
        # todo con numpy
        raw_image[np.isnan(raw_image)] = 0.625
        truth_matrix = np.full(raw_image.shape, False, dtype=bool)
        for i, row in enumerate(raw_image):
            for j, val in enumerate(row):
                truth_matrix[i, j] = 0.450 <= val <= 0.800

        tc = 20
        bc = 20

        izquierda = np.any(truth_matrix[tc:-bc, 0:213])
        centro = np.any(truth_matrix[tc:-bc, 213:427])
        derecha = np.any(truth_matrix[tc:-bc, 427:640])

        return Vector3(int(izquierda), int(centro), int(derecha))


if __name__ == "__main__":
    rospy.init_node('turtlebot_kinect')
    perception = Turtlebot_Perception()
    perception.send()
