#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = False


def dwa_control(x, config, goal, ob, count):
    """
    Dynamic Window Approach control
    """

    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob, count)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.6  # [m/s]   0.8
        self.min_speed = -0.4  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s] 100.0
        self.max_accel = 1  # [m/ss]
        self.max_dyawrate = 200.0 * math.pi / 180.0  # [rad/ss] 100.0
        self.dt = 0.1  # [s] Time tick for motion prediction 0.1
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.3
        self.speed_cost_gain = 0.7
        self.obstacle_cost_gain = 1         # initial: 1, 0.1, 1 ; version 1: 0.15 0.6 1
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.3  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check
        self.cnt = 0

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    motion model
      x = [x, y, orientation, v, w]
      u = [v, w]
    """

    x[2] += u[1] * dt   # change oritentation
    x[0] += u[0] * math.cos(x[2]) * dt  # position changing
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yaw_rate min, yaw_rate max]     dw = Vs jiao Vd
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    
    # give back a dynamic window
    return dw


def predict_trajectory(x_init, v, y, config):   
    """
    predict trajectory with an input
    Based on the current state x of the robot, a sequence of states x will be generated
    """

    x = np.array(x_init)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)    # x: 1x5
        traj = np.vstack((traj, x))     # vertical stack
        time += config.dt
        # a loop of state seq
    return traj


def calc_control_and_trajectory(x, dw, config, goal, ob, count):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")     # infinite
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):

            trajectory = predict_trajectory(x_init, v, y, config)   # traj

            # calc cost
            # TODO here
            # final_cost = to_goal_cost + speed_cost + ob_cost
           
            # calculate to_goal_cost, speed_cost and ob_cost
                  # use calc_to_goal_cost function to get angle cost  add distance?
            #speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            if (count<= 15):
                speed_cost = startspeedcostgain(config.max_speed/10 , trajectory[-1, 3],config.speed_cost_gain)
                to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)*3
                ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
                #print("start time")
            else:
                speed_cost = speedcostgain(config.max_speed , trajectory[-1, 3],config.speed_cost_gain)
                to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)    # use calc_obstacle_cost function to get ob cost
                ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            
            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost  
                best_u = [v, y]
                best_trajectory = trajectory

                best_to_goal_cost = to_goal_cost
                best_speed_cost = speed_cost
                best_ob_cost = ob_cost
    
    print('cost-goal,speed,obstacle', best_to_goal_cost, best_speed_cost, best_ob_cost)
    print('best_u (linear,angle) ', best_u)
    return best_u, best_trajectory

def speedcostgain(max_speed, end_speed , cfg):
    cost = 1
    delta = max_speed - end_speed
    if (delta < 0.1):
        cost = cfg * delta * 0.5
    elif(delta < max_speed*0.33):
        cost = cfg * delta * (1.5)
    elif(delta < (max_speed - 0.08)):
        cost = cfg * delta * (1.5)
    elif delta <= max_speed:
        cost = cfg * delta * (5)
    else:
        cost = cfg * delta* (5)

    return cost
def startspeedcostgain(max_speed, end_speed , cfg):
    cost = 1
    delta = max_speed - end_speed
    cost = cfg * delta

    return cost

def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
        THIS PART REFERS TO: https://zhuanlan.zhihu.com/p/90686873, AND EDIT BY yhy.
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]     # inspect all points in history trajectory
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy) 
    mini = np.min(r)
    if mini < 0.5:
        cost = 1.0 / mini
    elif mini > 1 :
        cost = 0.5 / mini                       # cal length of the hypotenuse
    else:
        cost = 1.0 / mini  # large min r is better, so take the countdown
    #print("np.min(r): ", np.min(r))
    
    # TODO here---------------------------------------------------------
    # judge whether the route will collide  ob_diff
    ort = trajectory[:, 2]  # all orientation
    parameters_matrix = np.array([[np.cos(ort), -np.sin(ort)], [np.sin(ort), np.cos(ort)]])   # save angles of all points in traj towards obs
    parameters_matrix = np.transpose(parameters_matrix, [2, 0, 1])
    ob_diff = ob[:, None] - trajectory[:, 0:2]
    ob_diff = ob_diff.reshape(-1, ob_diff.shape[-1])
    ob_dis = np.array([np.dot(ob_diff, x) for x in parameters_matrix])    # multiply
    #ob_dis = np.dot(ob_diff, parameters_matrix)
    ob_dis = ob_dis.reshape(-1, ob_dis.shape[-1])
    Len_lmt = config.robot_length / 2
    Wid_lmt = config.robot_width / 2

    #if np.min(r) <= 0.5:
    #   cost = 5.0/np.min(r)
    #if np.min(r) <= Len_lmt:
    #   cost = 10.0/np.min(r)
    #if np.min(r) <= Wid_lmt:
    #   return float("Inf")

    if (np.logical_and(np.logical_and(ob_dis[:, 0] <= Len_lmt, ob_dis[:, 1] <= Wid_lmt),
                        np.logical_and(ob_dis[:, 0] >= -Len_lmt, ob_dis[:, 1] >= -Wid_lmt))).any():
        return float("Inf")     # if collides, cost is infinit
    #-------------------------end-------------------------------------

    return cost  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
        The angle between the direction of travel and the target, the smaller the better
        x = [x, y, orientation, v, w]
    """
    cost = 0
    # TODO here-------------------------------------------------------
    # calculate cost to goal
    dx = goal[0] - trajectory[-1, 0]    # the last line of matrix is end point x
    dy = goal[1] - trajectory[-1, 1]    # end y
    #dist = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
    #print("dist: ",dist)
    diff = math.atan2(dy, dx) - trajectory[-1, 2]    # get orientation COST
    cost = abs(math.atan2(math.sin(diff), math.cos(diff)))  # cost function
    #----------------end---------------------------

    return cost
