#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv



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


class Robot():
    def __init__(self):
        rospy.init_node("reckoning_robot")
        

        # --------------------------------------------------------------------
        # CONSTANTS
        # --------------------------------------------------------------------
        self.dist_threshold = 0.01
        self.ang_threshold = np.pi / 180

        self.min_front_dist = 0.25 #[m] Distancia a la que se detiene del frente

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
        # self.real_pose_sub = rospy.Subscriber('real_pose', Pose,
        #                                       self.real_pose_fn)

        self.deph_sub = rospy.Subscriber('/camera/depth/image_raw',
                                           Image, self.process_depth)
        
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_color',
                                           Image, self.arrow_detector)
        

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
        # TIMER
        # --------------------------------------------------------------------
        self.period = 0.1
        rospy.Timer(rospy.Duration(self.period), self.publish_depth)


    def ang_actuation_fn(self, data: Float64):
        if self.stopped() and not self.arrow_rotation:
            self.ang_vel = 0
            self.vel = 0
        else:
            self.ang_vel = float(data.data)
            self.vel = 0.02
        # rospy.loginfo(f"Angular speed received: {self.ang_vel}")
        self.publish_vel()

        self.arrow_rotation = False





    def process_depth(self, data: Image):

        #Procesar distancia en la imagen con numpy
        depth_image = self.bridge.imgmsg_to_cv2(data)

        """
        Guardar imagenes para poder medir correctamente los pixeles pa cortar

        if self.img_guardadas<10:
            cv2.imwrite(f"imagen_prof{self.img_guardadas}.png", depth_image)
            print("imagen guardada")
        """

        #Left depth from image
        depth_left = depth_image[100:300,100:150]

        #Right depth from image
        depth_right = depth_image[100:300,-150:-100]

        #Center depth from image
        depth_center = depth_image[100:300,150:-150]

        prom_left = float(depth_left.mean())
        prom_right = float(depth_right.mean())
        prom_center = float(depth_center.mean())

        self.distance = np.array((prom_left, prom_right, prom_center))
        
        

    def publish_depth(self, data):
        # Publish difference between left and right distance 
        dif_distance= self.distance[0] - self.distance[1]
        self.wall_distance_state.publish(dif_distance)
        

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

    
    def stopped(self):
        #Podriamos añadirle caso en que ve la flecha suficientemente bien
        return self.distance[2] <= self.min_front_dist

    def arrow_detector(self,data):
        zeros=np.zeros(2)
        def pos_y_pendiente(pos):
            x1, y1, x2, y2 = pos 
            if x2-x1 == 0 or y2-y1 == 0:
                return zeros
            return np.array(x1, x2)
        #Deteccion de la flecha

        if self.stopped():
            self.arrow_rotation = True
            #el bridge es unico?
            img =  self.bridge.imgmsg_to_cv2(data)[100:300,:]
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            edges = cv.Canny(gray, 50, 150, apertureSize=3)
            lines_positions = cv.HoughLinesP(edges, 1, np.pi/180, 100,
                                    minLineLength=100, maxLineGap=10)[:,0]

            rectas = np.apply_along_axis(pos_y_pendiente, 1, lines_positions)
            rectas = rectas[rectas != zeros]
            
            #No se si esto esta bien
            if np.mean(rectas)< img.shape[1]/2:
                self.wall_distance_state.publish(np.pi/2)
            else:
                self.wall_distance_state.publish(-np.pi/2)
 





if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
