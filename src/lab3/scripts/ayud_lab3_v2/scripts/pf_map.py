#!/usr/bin/env python3
# fuente: https://github.com/mithi/particle-filter-prototype/tree/master/particle-filter-visualization

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler

class PFMap:
  def __init__(self, name):
    self.variables_init(name)
    self.get_map()
  

  def variables_init(self, name):
    self.name = name
    self.free_space_value = 254
    self.wall_space_value = 0

    
  def get_map(self):
    # obtenemos el mapa y sus datos
    map = rospy.wait_for_message(self.name, OccupancyGrid, timeout=None)
    self.width, self.height = map.info.width, map.info.height
    self.resolution = map.info.resolution
      
    # pasar el mapa a escala de 0 a 255
    map_np = 100 - np.array(map.data).reshape((self.height, self.width))
    self.np = (map_np * (255/100.0)).astype(np.uint8)# blanco = 254
  

  def get_free_positions(self):
    return self.get_positions_where(self.free_space_value)
  

  def get_walls_positions(self):
    return self.get_positions_where(self.wall_space_value)
  

  def get_positions_where(self, value):
    # creamos una lista con las posiciones libres
    pos_y, pos_x = np.where(self.np == value) # [y1,...], [x1,...]
    positions_pix = np.c_[pos_x.ravel(), pos_y.ravel()] # [[x1,y1]....]
    positions = positions_pix*self.resolution # pasamos de la posicion de pixeles a metros
    
    return positions


def pub_initial_pose(x,y,yaw):
  pub_init_pose = rospy.Publisher('initial_pose', Pose, queue_size=1)
  
  while pub_init_pose.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep(0.2)
  
  init_pose = Pose()
  init_pose.position.x = x
  init_pose.position.y = y
  
  x,y,z,w = quaternion_from_euler(0,0,yaw)
    
  init_pose.orientation.x = x
  init_pose.orientation.y = y
  init_pose.orientation.z = z
  init_pose.orientation.w = w

  pub_init_pose.publish(init_pose)
  
