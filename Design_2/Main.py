import math
import pygame
from Robot import Graphics, Robot, Ultrasonic

#Map constants
Map_dim = (600,1200)
Start = (100, 200)
Goal = (1000, 300)

#Environment graphics
gfx = Graphics(Map_dim, 'RobotDog.png', 'Map.png')

#The robot
robot = Robot(Start,0.01*2779.52)

#The sensor
sensor_range = 250, math.radians(40)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks()-last_time)/1000
    last_time = pygame.time.get_ticks()

    gfx.map.blit(gfx.map_img, (0,0))
    gfx.draw_map_necesities(Start, Goal)

    robot.kinematics(dt)
    gfx.draw_robot(robot.x,robot.y,robot.heading)
    point_cloud = ultra_sonic.sense_obstacles(robot.x,robot.y,robot.heading)
    robot.avoid_obstacles(point_cloud, dt)
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()

    #print(point_cloud)
