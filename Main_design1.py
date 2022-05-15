import math
import pygame
import RRTstar_planning
from Robot import Robot

running = True

#Map constants
Map_dim = (600,1200)
Start = (100, 200)
Goal = (1000, 300)

#RRT constants
winsize = [Map_dim[0], Map_dim[1]]
epsilon = 20.0
numnodes = 2000
radius = 15

#obstacle constants
#borders
object1 = pygame.Rect((0, 0), (0,1199))
object2 = pygame.Rect((0,585),(0,1199))
object3 = pygame.Rect((0,0),(16,599))
object4 = pygame.Rect((1183,0),(16,599))
#Bottom left obstacle
object5 = pygame.Rect((0,375),(220,225))
#Middle obstacle
object6 = pygame.Rect((535,0),(60,280))
#Bottom right obstacle
object7 = pygame.Rect((706,446),(380,153))

Obs = [object1,object2,object3,object4,object5,object6,object7]

#The robot
#robot = Robot(Start,0.01*2779.52)

#RRT implementation
success, screen, result = RRTstar_planning.product()

pygame.display.update()
pygame.event.clear()

robot = Robot(result)
pygame.draw.lines(screen, pygame.Color(150, 150, 150), closed=False, points=robot.desired_path, width=3)
time_prev = pygame.time.get_ticks()

if success:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # update map
        pygame.display.update()
        screen.blit(bg, (0, 0), special_flags=pygame.BLEND_ALPHA_SDL2)  # draw background
        pygame.draw.lines(screen, pygame.Color(150, 150, 150), closed=False, points=robot.desired_path, width=3)
        pygame.draw.circle(screen, (0, 0, 0), startR, 5, 10)
        pygame.draw.circle(screen, (0, 0, 0), goalR, 20, 3)
        draw_obstacles_list(screen, obstacles_list)

        # update robot
        robot.draw(map=screen)
        robot.move()

        # timing
        dt_tick = (pygame.time.get_ticks() - time_prev) / 1000  # difference divided by 1000 to get seconds
        time_prev = pygame.time.get_ticks()  # update time_prev
        pygame.time.delay(int((robot.dt - dt_tick) * 1000))

        # stop if finished
        if robot.finished:
            running = False