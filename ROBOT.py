import pygame
import math
import numpy as np

def distance (point1 , point2) :
    point1 = np.array(point1)
    
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self,startpos,width):
        self.m2p = 3779.52
        self.w = width
        self.x= startpos[0]
        self.y =startpos[1]
        self.heading =0
        
        self.vl = 0.01*self.m2p
        self.vr = 0.01*self.m2p
        
        self.maxspeed = 0.02*self.m2p
        self.minspeed = 0.01*self.m2p
        
        self.min_obs_dis = 100
        self.count_down = 5 
        
        
def avoidance_obs (self, point_cloud,dt):
    closest_osb = None
    dist = np.inf
    
    if len(point_cloud)> 1 :
        for point in point_cloud:
            if dist > distance([self.x,self.y],point):
                dist = distance ([self.x,self.y], point)
                closest_obs = (point,dist)
        
        if closest_obs[1] < self.min_obs_dist and self.count_down > 0 :
            self.count_down -= dt
            self.move_backward()
            
        else :
            self.count_down = 5
            self.moveforward()
            
def move_backward(self):
    self.vr = - self.minspeed
    self.vl = - self.minspeed/2
    
def move_forward(self):
    self.vr =  self.minspeed
    self.vl =  self.minspeed
    
def kinematics(self , dt):
    self.x+= ((self.vr+self.vl)/2)* math.cos(self.heading)*dt
    self.y-= ((self.vr+self.vl)/2)* math.sin(self.heading)*dt
    self.heading+=(self.vr - self.vl) / self.w *dt
    

        
    
