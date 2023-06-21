#!/usr/bin/env python3
# fuente: https://github.com/mithi/particle-filter-prototype/tree/master/particle-filter-visualization
import rospy
import numpy as np

from geometry_msgs.msg import PoseArray, Pose

from tf.transformations import quaternion_from_euler
from random import  randint, gauss, random

from particle import Particle


class ParticlesManager(object):
  
  def __init__(self):
    self.variables_init()
    self.connections_init()

  
  def variables_init(self):
    self.num_particles = 1200
    self.sigma = 0.01
  
    self.particles = []


  def connections_init(self):
    self.pub_particules = rospy.Publisher('particles', PoseArray, queue_size=10)
  
  
  def create_particles(self, free_positions):

    for i in range(0, self.num_particles):
      # elegimos una pocicion libre aleatoria
      pos = free_positions[randint(0, len(free_positions))]
      
      # pasamos de posiciones discretas a continua agregando incertidumbre gaussiana
      x, y, ang = pos[0] + gauss(0, self.sigma), pos[1] + gauss(0, self.sigma) , random()*2*np.pi 
     
      # creamos una nueva particula
      new_particule = Particle(x, y, ang, sigma = self.sigma)
      self.particles.append(new_particule)
    
    self.publish_particules()


  def update_particles(self, delta_x, delta_y, delta_ang):
  
      for particle in self.particles:
          particle.move(delta_x, delta_y, delta_ang)
        
      self.publish_particules()
  

  def publish_particules(self):
    # aqui creamos uno poseArray para mostra las particulas en Rviz
    pose_array_msg = PoseArray()
    pose_array_msg.header.frame_id = "map"

    for part in self.particles:
        part_pose = Pose()
        part_pose.position.x, part_pose.position.y = part.x, part.y
        quat = quaternion_from_euler(0,0, part.ang)

        part_pose.orientation.x = quat[0]
        part_pose.orientation.y = quat[1]
        part_pose.orientation.z = quat[2]
        part_pose.orientation.w = quat[3]

        pose_array_msg.poses.append(part_pose)

    pose_array_msg.header.stamp = rospy.Time.now()
    self.pub_particules.publish(pose_array_msg)


if __name__ == '__main__':
  from pf_map import PFMap, pub_initial_pose
  
  rospy.init_node('particle_manager')
  
  rospy.sleep(2)
  pub_initial_pose(x = 0.5, y = 0.5, yaw = 0.0)
  
  particle_manager = ParticlesManager()
  print("touch the screen with D key")
  map = PFMap("map")

  free_positions = map.get_free_positions()
  particle_manager.create_particles(free_positions)

  for i in range(0, 10):
    # rotate the particle 30 degrees
    particle_manager.update_particles(0, 0, (30*np.pi/180))
    rospy.sleep(2)
