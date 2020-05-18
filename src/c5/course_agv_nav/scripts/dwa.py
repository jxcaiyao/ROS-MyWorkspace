#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = False


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """

    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

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
        self.max_speed = 0.8  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 100.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1  # [m/ss]
        self.max_dyawrate = 100.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 1.4
        self.speed_cost_gain = 2.0
        self.obstacle_cost_gain = 1.0
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.3  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check

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
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
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

    #  [vmin, vmax, yaw_rate min, yaw_rate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    cost_mat = []
    sum_to_goal_cost = 0
    sum_speed_cost = 0
    sum_ob_cost = 0
    n = 0

    maxob = -float("inf")
    minob = float("inf")
    maxgo = -float("inf")
    mingo = float("inf")
    maxsp = -float("inf")
    minsp = float("inf")

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost

            to_goal_cost = calc_to_goal_cost(trajectory, goal)
            # to_goal_cost = 0

            speed_cost = calc_speed_cost(trajectory, goal)

            ob_cost = calc_obstacle_cost(trajectory, ob, config)
            # ob_cost = 0
            
            cost_mat.append([v, y, to_goal_cost, speed_cost, ob_cost])
            sum_to_goal_cost += to_goal_cost
            sum_speed_cost += speed_cost
            sum_ob_cost += ob_cost
            n += 1

            if ob_cost > maxob:
                maxob = ob_cost
            if ob_cost < minob:
                minob = ob_cost

            if to_goal_cost > maxgo:
                maxgo = to_goal_cost
            if to_goal_cost < mingo:
                mingo = to_goal_cost

            if speed_cost > maxsp:
                maxsp = speed_cost
            if speed_cost < minsp:
                minsp = speed_cost
            # TODO here

    avg_to_goal_cost = sum_to_goal_cost / n
    avg_speed_cost = sum_speed_cost / n
    avg_ob_cost = sum_ob_cost / n


    for vy_cost in cost_mat:
        final_cost = config.to_goal_cost_gain  * vy_cost[2] / avg_to_goal_cost              \
                   + config.speed_cost_gain    * vy_cost[3] / avg_speed_cost                \
                   + config.obstacle_cost_gain * vy_cost[4] / avg_ob_cost
        
        if min_cost >= final_cost:
            min_cost = final_cost
            best_u = [vy_cost[0], vy_cost[1]]
            best_trajectory = predict_trajectory(x_init, vy_cost[0], vy_cost[1], config)

    print("ob_cost:")
    print((maxob-minob)/avg_ob_cost)
    print("go_cost:")
    print((maxgo-mingo)/avg_to_goal_cost)
    print("sp_cost:")
    print((maxsp-minsp)/avg_speed_cost)
    print("\n")
    # print(min_cost)

    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
    """
    # # TODO here

    minr = float("inf")
    mincost = float("inf")
    for i in range(len(ob)):
        j = 0
        while j < len(trajectory):
            dx = ob[i,0] - trajectory[j,0]
            dy = ob[i,1] - trajectory[j,1]
            da = math.atan2(dy,dx) - trajectory[j,2]

            cost = (math.sqrt(dx*dx+dy*dy) * (1.4-fsgn(trajectory[j,3])*math.cos(da)))
            if cost < mincost:
                mincost = cost
            j += 6

    mincost = 1 / mincost 
    return mincost  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """
    cost = 0
    # TODO here

    dx = goal[0] - trajectory[-1,0]
    dy = goal[1] - trajectory[-1,1]
    da = math.atan2(dy,dx) - trajectory[-1,2]
    # print(goal)

    cost = math.sqrt(dx*dx+dy*dy)*(1.5-fsgn(trajectory[-1,3]/0.15)*math.cos(da))
    
    return cost

def calc_speed_cost(trajectory, goal):
    v = trajectory[-1,3]
    y = trajectory[-1,4]
    cost = (-abs(v) + 0.2 * abs(y)) + 0.8

    return cost

def sgn(num):
    if num > 0:
        return 1
    else:
        return -1

def fsgn(num):
    return math.atan(num)/1.5708