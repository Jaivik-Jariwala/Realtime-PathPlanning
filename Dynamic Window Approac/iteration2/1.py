import numpy as np 
#  Initial (IVPs) and Boundary Value Problems (BVPs) DSolve can be used for finding the general solution to a differential equation or system of differential equations. The general solution gives information about the structure of the complete solution space for the problem.
from scipy.integrate import solve_ivp 

import matplotlib.animation as ani
import matplotlib.pyplot as plt 

#change this to bird-eye generated map for realtime path planning 
from DWA.iteration2.animation import animation 

import math

# class define Path
class Path:
# function  for the path variables
    def __init__(self, a_v, l_v):
        self.x = None  # x coordinates
        self.y = None     # y coordinates 
        self.a = None      # a angle of path 
        self.a_v = a_v  # a_v angular velocity 
        self.l_v = l_v  # l_v linear velocity 

class Obstacle():
    def __init__(self, x, y, size):
        self.x = x          # x coordinate of obstacle
        self.y = y          # y coordinate of obstacle
        self.size = size    # size of obstacle

# class define the bot
class Robot:
    def __init__(self, init_x, init_y, init_a):
    
        self.x = init_x 
        self.y = init_y 
        self.a = init_a 
        
        #constant angles
        self.a_v = 0.0 
        self.l_v = 0.0  


        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_a = [init_a]

        self.traj_a_v = [0.0]
        self.traj_l_v = [0.0]

    def update_state(self, a_v, l_v, dt):

        self.a_v = a_v
        self.l_v = l_v

        next_x = self.l_v * math.cos(self.a) * dt + self.x
        next_y = self.l_v * math.sin(self.a) * dt + self.y 
        next_a = self.a_v * dt + self.a 

        self.traj_x.append(next_x)
        self.traj_y.append(next_y)
        self.traj_a_v.append(next_a)

        return self.x, self.y, self.a

class
