from random import gauss

class Particle(object):
    
  def __init__(self, x, y, ang, sigma = 0.1):
    self.x, self.y, self.ang = x, y, ang
    self.last_x, self.last_y, self.last_ang = x, y, ang
    self.sigma = sigma
               
  def move(self, delta_x, delta_y, delta_ang):
    # movemos las particulas acompanadas de un grado de incertesa gaussiano
    self.x += delta_x +  gauss(0, self.sigma) 
    self.y += delta_y + gauss(0, self.sigma)
    self.ang += delta_ang + gauss(0, self.sigma)
  
  def pos(self):
    return [self.x, self.y, self.ang]
  
  def last_pos(self):
    return [self.last_x, self.last_y, self.last_ang]

