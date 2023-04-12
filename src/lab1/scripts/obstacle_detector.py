import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge
import numpy as np

class Turtlebot_Kinect( object ):
    def __init__( self ) -> None:
        self.raw_sub = rospy.Subscriber( 'camera/depth/image_raw', Image, self.raw_cb )
        self.bridge = CvBridge()
        self.current_cv_raw_image = None

    def raw_cb( self, data: Image ) -> None:
        self.current_cv_raw_image = self.bridge.imgmsg_to_cv2( data )

class Turtlebot_Perception( object ):
    def __init__( self ) -> None:
        self.pub = rospy.Publisher( '/occupancy_state', Vector3, queue_size=10 )
        self.rate_obj = rospy.Rate( 5 )
        self.kinect_obj = Turtlebot_Kinect()

    
    def send( self ) -> None:
        while not rospy.is_shutdown():
            detected_objects = self.detect_objects()
            rospy.loginfo( f'publishing object detection {detected_objects}' )
            self.pub.publish( detected_objects )
            self.rate_obj.sleep()

    def detect_objects( self ) -> Vector3:
        raw_image = self.kinect_obj.current_cv_raw_image

        # revisar pixeles 0-213, 213-427, 427-640
        # cortar bordes abajo y arriba de matriz
        # revisar si hay objetos entre 450mm y 800mm
        # todo con numpy
        true_matrix = np.array([[True if 450 <= col <= 800 else False for col in row] for row in raw_image])
        
        izquierda = np.any(true_matrix[:, :213])
        centro = np.any(true_matrix[:, 213:427])
        derecha = np.any(true_matrix[:, 427:640])

        return Vector3((izquierda, centro, derecha))
            

if __name__ == "__main__":
    rospy.init_node( 'turtlebot_kinect' )
    perception = Turtlebot_Perception()
    perception.send()