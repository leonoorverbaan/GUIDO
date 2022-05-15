# -*- coding: utf-8 -*-
"""
Created on Thu Nov 25 17:16:14 2021

@author: timdadum
"""

import numpy as np
import pygame
from math import e
from scipy.ndimage import gaussian_filter1d

"""PARAMETERS"""
h = 0.1  # m
D = 0.5  # m
L = D
lr = L / 2
m = 10  # kg
delta_max = np.pi / 4

"""DEFINE SIMULATION VARIABLES AND FUNCTIONS"""


class Robot:
    def __init__(self, path):
        # unpack path
        self.theta_desired = None
        self.desired_path = None
        self.unpack(path)

        # define state
        self.state = np.array([self.desired_path[0][0],  # x position
                               self.desired_path[0][1],  # y position
                               self.theta_desired[0],  # yaw angle
                               0])  # steering angle
        self.past_states = np.array([self.state[0],
                                     self.state[1],
                                     self.state[2],
                                     self.state[3]])
        self.past_velocities = np.array([])
        self.finished = False  # indicates whether the robot has finished
        self.reached_goal = False  # indicates whether the robot has reached the goal
        self.phi = 0  # rad, input steering angle [rad]
        self.v = 1  # input velocity [m/s]
        self.v_min = 1  # minimal velocity [m/s]
        self.v_max = 15  # maximum velocity [m/s]

        # time instances
        self.T = 0  # elapsed time [s]
        self.dt = 0.01  # time step [s]

        # data instances
        self.accumulated_error = 0  # accumulated ye [m]
        self.errors = np.array([])  # array of errors [m]

        # visualizing robot img
        self.img = pygame.image.load("RobotDog.png")
        self.img = pygame.transform.scale(self.img, (40, 30))  # rescale image
        self.img_rotated = self.img
        self.rect = self.img_rotated.get_rect(center=(self.state[0], self.state[1]))

        # steering characteristics
        self.delta_max = np.pi / 2
        self.kt = 0.0040
        self.ke = 0.0014

    def unpack(self, path):
        desired_x = np.array([])
        desired_y = np.array([])
        for point in path:
            x, y = point
            desired_x = np.append(desired_x, x)
            desired_y = np.append(desired_y, y)

        # smoothen line
        desired_x = gaussian_filter1d(desired_x, 3)
        desired_y = gaussian_filter1d(desired_y, 3)

        # assign values
        self.theta_desired = calc_theta_des(desired_x, desired_y)
        self.desired_path = [(desired_x[i], desired_y[i]) for i in range(len(desired_x))]

    def draw(self, map):
        map.blit(self.img_rotated, self.rect)

    def move(self, dt=0.1):
        """This method is only for testing the visualization of the robot"""
        self.dt = dt
        self.step()

        # update image
        self.img_rotated = pygame.transform.rotozoom(self.img, np.rad2deg(self.state[2]), 1)
        self.rect = self.img_rotated.get_rect(center=(self.state[0], self.state[1]))

    def write_state(self):
        """Write states to class attribute for post-processing"""
        self.past_states = np.vstack([self.past_states, self.state])

    def get_index(self):
        """Returns the current simulation index for array indexing"""
        return round(self.T / self.dt)

    def step(self):
        """Updates the simulation by one time step"""
        print(f'Timestep {round(self.T / self.dt)}')

        # calculate error and desired position
        ye, desired_position, desired_index = self.calculate()

        # add error to data
        self.errors = np.append(self.errors, ye)

        # change input according to calculations
        self.control(ye, desired_index)

        # update state
        dx = self.get_xdot(self.v) * self.dt
        dy = - self.get_ydot(self.v) * self.dt
        dtheta = self.get_theta_dot(self.v, L) * self.dt
        ddelta = self.phi * self.dt

        # # check if delta complies constraints
        self.state = limit_state(np.add(self.state, np.array([dx, dy, dtheta, ddelta])),
                                 np.array([None, None, None, self.delta_max]))

        print(f"states: {self.state}")

        # write state to past_states array
        self.write_state()

        # update elapsed time
        self.T += self.dt

        if eucl(self.desired_path[-1], self.get_position()) < 30:
            self.finished = True
            self.reached_goal = True

    def get_position(self):
        return self.state[0], self.state[1]

    def calculate(self):
        """Returns the distance between the robot CoM and the closest point of the
        desired trajectory (see slide 21 of lecture 8 of RD&C for more
        information). Taking the closest point ensures orthogonality between
        this distance line and the trajectory."""

        # initialize as dummy values
        ye = float('inf')
        desired_position = None
        desired_index = None

        # go through all points to find closest point. Keep absolute value of error and direction towards desired point
        for count, point in enumerate(self.desired_path):
            dist = eucl(point, self.get_position())
            if dist < ye:
                ye = dist
                desired_position = point
                desired_index = count

        # return distance and desired position
        return ye, desired_position, desired_index

    def get_rotation(self, desired_index):
        """Returns absolute error in theta"""
        theta = self.state[2]
        theta_des = self.theta_desired[desired_index]
        return rotation(theta_des, theta)

    def control(self, ye, desired_index):
        """Updates control values through the Stanley Algorithm"""
        # determine errors. ye is the cross-tracking error, theta_e is the error in angle
        theta_e, theta_direction = self.get_rotation(desired_index)

        # define points a and b for determining direction
        a = self.desired_path[desired_index]
        b = self.desired_path[desired_index - 1] if desired_index != 0 else self.desired_path[desired_index]

        # determine the steering angle we would like to have based on errors and side we're on
        delta_ye = self.ke * (1 if ye_direction(a, b, self.get_position()) else -1) * ye
        delta_theta = self.kt * theta_direction * theta_e

        delta_des = limit(delta_theta, self.delta_max) + limit(delta_ye, self.delta_max)
        delta_error = delta_des - self.state[3]

        # determine phi (change of steering angle) and v based on the error in our steering angle
        self.phi = np.pi / (0.2 * self.delta_max) * delta_error
        self.v = self.v_min + (self.v_max - self.v_min) * e ** (-5 * theta_e)
        self.past_velocities = np.append(self.past_velocities, self.v)

    def get_xdot(self, v):
        return v * np.cos(self.state[2] + self.get_beta())

    def get_ydot(self, v):
        return v * np.sin(self.state[2] + self.get_beta())

    def get_theta_dot(self, v, L):
        return v * limit(np.tan(self.state[3]), 10) * np.cos(self.get_beta()) / L

    def get_beta(self):
        return np.arctan2(lr * np.tan(self.state[3]), L)

def eucl(p_des, p):
    """Returns eucledian distance between points A (tuple, desired) and B (tuple, actual)"""
    # unpacking tuples
    (xdes, ydes) = p_des
    (x, y) = p

    # determine errors
    xe = xdes - x
    ye = ydes - y

    # calculate and return eucledian
    return np.sqrt(xe ** 2 + ye ** 2)

def ye_direction(a, b, p):
    """Takes the determinant of the position p relative to a tangent line from point a to point b (!) such that the
    Returns a bool isLeft. Points should be indexable (numpy array, list, etc.).
    a: start point
    b: end point
    p: position robot
    TRUE = ROBOT IS RIGHT, TURN LEFT FALSE = ROBOT IS LEFT, TURN RIGHT"""
    (ax, ay) = a
    (bx, by) = b
    (px, py) = p

    return ((px - ax) * (ay - by) - (ay - py) * (bx - ax)) < 0

def limit(val, max_abs):
    """Takes as input a number and the maximum absolute value that number should have and limits the value in that range.
    For example, the steering rate should be between -pi/4 and +pi/4, so we call limit(delta, pi/4). This function then
    returns a limited value."""
    if val < -max_abs:
        return -max_abs
    elif val > max_abs:
        return max_abs
    else:
        return val

def limit_state(state, state_limits):
    """Limit state vector to given state bi-directional values"""
    reduced_state = state
    for index, value in enumerate(reduced_state):
        if state_limits[index] is not None:
            reduced_state[index] = limit(value, state_limits[index])
    return reduced_state

def rotation(alpha, beta):
    """Takes as input two angles from 0 to 2pi radians and returns the shortest rotation and rotation direction to allign
    beta with alpha (not vice versa)"""
    gamma = (beta - alpha) % (2 * np.pi)

    if gamma > np.pi:
        return np.pi - (gamma - np.pi), 1
    else:
        return gamma, -1

def calc_theta_des(x_des, y_des):
    """"Takes as input an xy-trajectory and calculates the corresponding desired
    tangent (theta) over the same range as x and y."""
    try:
        theta_des = np.zeros(len(x_des))

        for i in range(len(x_des) - 1):
            theta_des[i] = np.arctan2(-(y_des[i + 1] - y_des[i]), x_des[i + 1] - x_des[i])

        # correct for indexing issues (taking tangent requires two points, this may cause a mismatch in index.
        theta_des[-1] = np.arctan2(y_des[-1] - y_des[-2], x_des[-1] - x_des[-2])

        return np.where(theta_des > 0, theta_des, theta_des + 2 * np.pi)
    except IndexError:
        print('Arguments x and y should be of equal length')
