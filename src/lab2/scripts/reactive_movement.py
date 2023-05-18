#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
import numpy as np
from cv_bridge import CvBridge
import cv2



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
        self.bridge = CvBridge()
        self.img_guardadas=0

        # --------------------------------------------------------------------
        # CONSTANTS
        # --------------------------------------------------------------------
        self.dist_threshold = 0.01
        self.ang_threshold = np.pi / 180
        

        # --------------------------------------------------------------------
        # INITIAL CONDITIONS
        # --------------------------------------------------------------------
        self.distance = np.array((0.0, 0.0))
        self.ang = 0.0

        self.vel = 0.0
        self.ang_vel = 0.0

        self.rotating = False
        self.stop = False

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

        self.kinect_sub = rospy.Subscriber('/camera/depth/image_raw',
                                           Image, self.process_depth)


        # --------------------------------------------------------------------
        # WALL DISTANCE P CONTROL
        # --------------------------------------------------------------------
        self.wall_set_point = rospy.Publisher('/wall_distance/setpoint',
                                             Float64, queue_size=1)
        rospy.loginfo("Waiting wall distance pid setpoint process")
        while self.wall_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

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






    def accion_mover(self, pose_array: PoseArray):
        for pose in pose_array.poses:
            self.goal_pos = np.array((pose.position.x, pose.position.y))
            goal_vec = self.goal_pos - self.pos
            ang_diff = (np.arctan2(goal_vec[1], goal_vec[0]) - self.ang)

            # Girar
            # Esperar a que este alineado con goal
            self.ang_set_point.publish(0)
            rospy.loginfo("Waiting for alignment")
            self.rotating = True
            while abs(ang_diff) > self.ang_threshold:
                rospy.sleep(self.period)
                ang_diff = (np.arctan2(goal_vec[1], goal_vec[0]) - self.ang)
            self.rotating = False

            # Moverse hasta estar cerca
            self.dist_set_point.publish(0)
            rospy.loginfo("Start movement")
            while np.linalg.norm(goal_vec) > self.dist_threshold:
                rospy.sleep(self.period)
                goal_vec = self.goal_pos - self.pos

        self.stop = True






    def ang_actuation_fn(self, data: Float64):
        if self.stop:
            self.ang_vel = 0
        else:
            self.ang_vel = float(data.data)
        # rospy.loginfo(f"Angular speed received: {self.ang_vel}")
        self.publish_vel()




    def process_depth(self, data: Image):


        depth_image = self.bridge.imgmsg_to_cv2(data)

        if self.img_guardadas<10:
            cv2.imwrite(f"imagen_prof{self.img_guardadas}.png", depth_image)
            print("imagen guardada")

        #Left depth from image
        depth_left = depth_image[100:300,100:150]

        #Right depth from image
        depth_right = depth_image[100:300,-150:-100]


        

        return(0)







        
        pass
        #Procesar distancia en la imagen con numpy

    def publish_depth(self, data):
        # Publish difference between left and right distance 
        dif_distance= self.distance[0] - self.distance[1]
        self.wall_distance_state.publish(dif_distance)
        
        pass

    def publish_vel(self):
        # Publish odometry to self.dist_state
        velocity = Twist()
        velocity.linear.x = self.vel
        velocity.angular.z = self.ang_vel
        self.vel_applier.publish(velocity)

    
    def stopper(self):

        if self.flecha_detector():
            self.stop=True

    def flecha_detector(self):

        #Deteccion de la flecha

        #color + forma + distancia


        pass

        





if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
