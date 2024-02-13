'''
DWA is determined as follows:

1. Determine possible combinations of $(v, \omega)$ Decide on the combination of
2. Create a predicted trajectory
3. Substitute into the evaluation function and select $(v, \omega)$ that produced the best trajectory selected
4. Assign the selected $(v, \omega)$ to the robot Assign to robot
5. repeat until you reach the goal
'''

import numpy as np
np.seterr(divide='ignore', invalid='ignore')
import matplotlib.pyplot as plt
import pandas as pd
from animation import Animation_robot
import math
import sys

"""
    Perform min-max normalization on the input data.

    Parameters:
    data (array-like): The input data to be normalized.

    Returns:
    array: The normalized data.

    Explanation:
    Min-max normalization rescales the data to a fixed range [0, 1].
    For each value in the data:
    - Subtract the minimum value from it.
    - Divide the result by the range (difference between maximum and minimum values).

    If the range is zero, it implies that all values are identical.
    In this case, the function sets all values to 0.0 to avoid division by zero.

    Note: The function uses NumPy for efficient array operations.
"""
def min_max_normalize(data):
    data = np.array(data)
    max_data = max(data)
    min_data = min(data)
    if max_data - min_data == 0:
        data = [0.0 for i in range(len(data))]
    else:
        data = (data - min_data) / (max_data - min_data)
    return data

"""
    Correct the angle to be within the range -π to π.

    Parameters:
    angle (float): The input angle to be corrected.

    Returns:
    float: The corrected angle.

    Explanation:
    This function ensures that the input angle is within the range -π to π
    by adding or subtracting multiples of 2π until it falls within this range.
"""
def angle_range_corrector(angle):

    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi

    return angle

"""
    Draw a circle centered at (center_x, center_y) with a line indicating an angle.

    Parameters:
    center_x (float): X-coordinate of the center of the circle.
    center_y (float): Y-coordinate of the center of the circle.
    angle (float): Angle in radians to indicate the direction.
    circle_size (float, optional): Radius of the circle. Defaults to 0.2.

    Returns:
    tuple: Lists of X and Y coordinates for the circle and line.
"""
def write_circle(center_x, center_y, angle, circle_size=0.2):
    circle_x = [] 
    circle_y = [] 

    steps = 100 
    for i in range(steps):
        circle_x.append(center_x + circle_size*math.cos(i*2*math.pi/steps))
        circle_y.append(center_y + circle_size*math.sin(i*2*math.pi/steps))

    circle_line_x = [center_x, center_x + math.cos(angle) * circle_size]
    circle_line_y = [center_y, center_y + math.sin(angle) * circle_size]

    return circle_x, circle_y, circle_line_x, circle_line_y

# path class
class Path():
    def __init__(self, u_th, u_v): 
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th

# obstacle class
class Obstacle():
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size

# robot class
'''
Update the state of the robot.
Parameters:
u_th (float): Angular velocity control input.
u_v (float): Linear velocity control input.
dt (float): Time step.
Returns:
tuple: Updated x, y, and theta coordinates.
        
Predict the future state of the robot.
Parameters:
ang_velo (float): Angular velocity.
velo (float): Linear velocity.
x (float): Current x-coordinate.
y (float): Current y-coordinate.
th (float): Current orientation.
dt (float): Time step.
pre_step (int): Number of steps to predict.
Returns:
tuple: Lists of predicted x, y, and theta coordinates.
'''

class Robot():
    def __init__(self, init_x, init_y, init_th):
        
        self.x = init_x
        self.y = init_y
        self.th = init_th
        self.u_v = 0.0
        self.u_th = 0.0

        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_th = [init_th]
        self.traj_u_v = [0.0]
        self.traj_u_th = [0.0]

    def update_state(self, u_th, u_v, dt): 

        self.u_th = u_th
        self.u_v = u_v

        next_x = self.u_v * math.cos(self.th) * dt + self.x
        next_y = self.u_v * math.sin(self.th) * dt + self.y
        next_th = self.u_th * dt + self.th

        self.traj_x.append(next_x)
        self.traj_y.append(next_y)
        self.traj_th.append(next_th)

        self.x = next_x
        self.y = next_y
        self.th = next_th

        return self.x, self.y, self.th # stateを更新

# Class Simulation_bot
'''Acceleration limits'''
'''predict future movement bot'''
'''
Predict the future state of the robot.
Parameters:
ang_velo (float): Angular velocity.
velo (float): Linear velocity.
x (float): Current x-coordinate.
y (float): Current y-coordinate.
th (float): Current orientation.
dt (float): Time step.
pre_step (int): Number of steps to predict.
Returns:
tuple: Lists of predicted x, y, and theta coordinates.'''

class Simulation_bot(): # DWAのシミュレータ用
    def __init__(self):
        # self.model 独立二輪型
        # 加速度制限
        self.max_accelation = 1.0
        self.max_ang_accelation = 100 * math.pi /180
        # 速度制限
        self.lim_max_velo = 1.6 # m/s
        self.lim_min_velo = 0.0 # m/s
        self.lim_max_ang_velo = math.pi
        self.lim_min_ang_velo = -math.pi

    # 予想状態を作成する
    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step): # DWA用(何秒後かのstateを予測))
        next_xs = []
        next_ys = []
        next_ths = []

        for i in range(pre_step):
            temp_x = velo * math.cos(th) * dt + x
            temp_y = velo * math.sin(th) * dt + y
            temp_th = ang_velo * dt + th

            next_xs.append(temp_x)
            next_ys.append(temp_y)
            next_ths.append(temp_th)

            x = temp_x
            y = temp_y
            th = temp_th

        # print('next_xs = {0}'.format(next_xs))

        return next_xs, next_ys, next_ths # 予想した軌跡

# DWA algo class
class DWA():
    def __init__(self):
        #simulation bot
        self.simu_robot = Simulation_bot()

        # prediction time
        self.pre_time = 3
        self.pre_step = 30

        # control setup size
        self.delta_velo = 0.02
        self.delta_ang_velo = 0.02

        # sampling time
        self.samplingtime = 0.1

        # weights for evaluation
        self.weight_angle = 0.04
        self.weight_velo = 0.2
        self.weight_obs = 0.1

        #store all paths
        self.traj_paths = []
        self.traj_opt = []



        # calculate control input
    def calc_input(self, g_x, g_y, state, obstacles): 
        # Path  Planning
        paths = self._make_path(state)
        # Path評価
        opt_path = self._eval_path(paths, g_x, g_y, state, obstacles)

        self.traj_opt.append(opt_path)

        return paths, opt_path

    def _make_path(self, state): 
        # 
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)

      
        paths = []


        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):

                path = Path(ang_velo, velo)

                next_x, next_y, next_th \
                    = self.simu_robot.predict_state(ang_velo, velo, state.x, state.y, state.th, self.samplingtime, self.pre_step)

                path.x = next_x
                path.y = next_y
                path.th = next_th

                paths.append(path)

        self.traj_paths.append(paths)

        return paths

    def _calc_range_velos(self, state):
       
        range_ang_velo = self.samplingtime * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo

        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo

        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        range_velo = self.samplingtime * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo

        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo

        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo

    def _eval_path(self, paths, g_x, g_y, state, obastacles):

        nearest_obs = self._calc_nearest_obs(state, obastacles)

        score_heading_angles = []
        score_heading_velos = []
        score_obstacles = []

        for path in paths:
            # (1) heading_angle
            score_heading_angles.append(self._heading_angle(path, g_x, g_y))
            # (2) heading_velo
            score_heading_velos.append(self._heading_velo(path))
            # (3) obstacle
            score_obstacles.append(self._obstacle(path, nearest_obs))

        # print('angle = {0}'.format(score_heading_angles))
        # print('velo = {0}'.format(score_heading_velos))
        # print('obs = {0}'.format(score_obstacles))

        for scores in [score_heading_angles, score_heading_velos, score_obstacles]:
            scores = min_max_normalize(scores)

        score = 0.0
   
        for k in range(len(paths)):
            temp_score = 0.0

            temp_score = self.weight_angle * score_heading_angles[k] + \
                         self.weight_velo * score_heading_velos[k] + \
                         self.weight_obs * score_obstacles[k]

            if temp_score > score:
                opt_path = paths[k]
                score = temp_score

        return opt_path

    def _heading_angle(self, path, g_x, g_y): 

        last_x = path.x[-1]
        last_y = path.y[-1]
        last_th = path.th[-1]

        angle_to_goal = math.atan2(g_y-last_y, g_x-last_x)

        score_angle = angle_to_goal - last_th

        score_angle = abs(angle_range_corrector(score_angle))


        score_angle = math.pi - score_angle

        # print('score_sngle = {0}' .format(score_angle))

        return score_angle

    def _heading_velo(self, path): 

        score_heading_velo = path.u_v

        return score_heading_velo

    def _calc_nearest_obs(self, state, obstacles):
        area_dis_to_obs = 5
        nearest_obs = []

        for obs in obstacles:
            temp_dis_to_obs = math.sqrt((state.x - obs.x) ** 2 + (state.y - obs.y) ** 2)

            if temp_dis_to_obs < area_dis_to_obs :
                nearest_obs.append(obs)

        return nearest_obs

    def _obstacle(self, path, nearest_obs):
       
        score_obstacle = 2
        temp_dis_to_obs = 0.0

        for i in range(len(path.x)):
            for obs in nearest_obs: 
                temp_dis_to_obs = math.sqrt((path.x[i] - obs.x) * (path.x[i] - obs.x) + (path.y[i] - obs.y) *  (path.y[i] - obs.y))

                if temp_dis_to_obs < score_obstacle:
                    score_obstacle = temp_dis_to_obs 

                # そもそも中に入ってる判定
                if temp_dis_to_obs < obs.size + 0.75: 
                    score_obstacle = -float('inf')
                    break

            else:
                continue

            break

        return score_obstacle

class Const_goal():
    def __init__(self):
       
        self.traj_g_x = []
        self.traj_g_y = []

    def calc_goal(self, time_step): 
        if time_step <= 100:
            g_x  = 10.0
            g_y = 10.0
        else:
            g_x = -10.0
            g_y = -10.0

        self.traj_g_x.append(g_x)
        self.traj_g_y.append(g_y)

        return g_x, g_y

class Main_controller():
    def __init__(self):
        self.robot = Robot(0.0, 0.0, 0.0)
        self.goal_maker = Const_goal()
        self.controller = DWA()

       
        self.obstacles = []
        '''
        obstacle_num = 3
        for i in range(obstacle_num):
            # x = np.random.randint(-5, 5)
            # y = np.random.randint(-5, 5)
            # size = np.random.randint(-5, 5)

            self.obstacles.append(Obstacle(x, y, size))
        '''

        self.obstacles =[Obstacle(4, 1, 0.25), Obstacle(0, 4.5, 0.25),  Obstacle(3, 4.5, 0.25), Obstacle(5, 3.5, 0.25),  Obstacle(7.5, 9.0, 0.25)]

       
        self.samplingtime = 0.1

    def run_to_goal(self):
        goal_flag = False
        time_step = 0

        while not goal_flag:
        # for i in range(250):
            g_x, g_y = self.goal_maker.calc_goal(time_step)

        
            paths, opt_path = self.controller.calc_input(g_x, g_y, self.robot, self.obstacles)

            u_th = opt_path.u_th
            u_v = opt_path.u_v

     
            self.robot.update_state(u_th, u_v, self.samplingtime)

 
            dis_to_goal = np.sqrt((g_x-self.robot.x)*(g_x-self.robot.x) + (g_y-self.robot.y)*(g_y-self.robot.y))
            if dis_to_goal < 0.5:
                goal_flag = True

            time_step += 1

        return self.robot.traj_x, self.robot.traj_y, self.robot.traj_th, \
                self.goal_maker.traj_g_x, self.goal_maker.traj_g_y, self.controller.traj_paths, self.controller.traj_opt, self.obstacles

def main():
    animation = Animation_robot()
    animation.fig_set()

    controller = Main_controller()
    traj_x, traj_y, traj_th, traj_g_x, traj_g_y, traj_paths, traj_opt, obstacles = controller.run_to_goal()

    ani = animation.func_anim_plot(traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y, traj_opt, obstacles)

if __name__ == '__main__':
    main()

