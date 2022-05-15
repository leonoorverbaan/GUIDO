# Pygame Robot Dog design
# Leonoor Verbaan and Shanti Verschoor
# First person perspective start orientating in real life how a blind person navigates
# Think about correcting feedback (feedback that pushes you back)
# This file contains the robot module which contains a programme of the differential drive
# and it's behavior in the environment.


import pygame
import math
import numpy as np

"""PARAMETERS"""
h = 0.1  # m
D = 0.5  # m
L = D
lr = L / 2
m = 10  # kg
delta_max = np.pi / 4

#Calculate Eucledian distance between two points in the environment
def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)


class Robot:
    def __init__(self, startpos, width):

        self.m2p = 3779.52 #from meters to pixels
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        #Right and left wheel velocities
        self.vl = 0.01*self.m2p #cm/s
        self.vr = 0.01*self.m2p #cm/s
        self.maxspeed = 0.02*self.m2p
        self.minspeed = 0.01*self.m2p

        #Minimum distance from the robot to the obstacle, can be changed accordingly to our haptic device
        self.min_obs_dist = 100 #pixels
        self.count_down = 5 #seconds

    #Obstacle avoidance
    def avoid_obstacles(self,point_cloud, dt):
        closest_obst = None
        dist = np.inf

        if len(point_cloud) > 1: #When data is arriving from the sensor
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x,self.y], point)
                    closest_obs = (point, dist)

            if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                self.count_down -=dt
                self.move_backward()
            else:
                # reset count down
                self.count_down = 5

                self.move_forward()

    def move_backward(self):
        #change wheel speed, make robot go back in a circle trajectory
        self.vr = - self.minspeed
        self.vl = - self.minspeed/2

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def kinematics(self, dt):

        self.x += ((self.vl+self.vr)/2)*math.cos(self.heading)*dt
        self.y -= ((self.vl+self.vr)/2)*math.sin(self.heading)*dt #it is inversed because of the inversed world frame of the screen
        self.heading += (self.vr - self.vl)/self.w * dt

        if self.heading>2*math.pi or self.heading<-2*math.pi:
            self.heading = 0

        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vr), self.minspeed)



class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        #Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)

        # ----------MAP----------

        #load img
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)

        #dimensions
        self.height, self.width = dimensions


        #window settings
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0,0))


    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x,y))
        self.map.blit(rotated,rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)


    def draw_map_necesities(self, start,goal):
        pygame.draw.circle(self.map, self.green, start, 5, 0)
        pygame.draw.circle(self.map, self.green, goal, 5, 0)


class Ultrasonic:

    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self,x,y,heading):
        obstacles = []
        x1,y1 = x, y

        #the heading of the robot +- the sensor heading range
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 10, False):
            x2 = x1 + self.sensor_range[0]*math.cos(angle)
            y2 = y1 - self.sensor_range[0]*math.sin(angle)

            for i in range(0,100):
                u = i/100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x,y))
                    self.map.set_at((x,y), (0,208,255))
                    if (color[0], color[1], color[2]) == (0,0,0):
                        obstacles.append([x,y])
                        break
        return obstacles





