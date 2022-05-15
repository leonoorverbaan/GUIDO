import csv
import pygame
from pygame.locals import *
import sys, random
from math import sqrt, cos, sin, atan2, atan
from Robot import Robot
import numpy as np
from random import randint

#Map constants
Map_dim = (1200,600)
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

obs = [object1,object2,object3,object4,object5,object6,object7]
obstacle_list = obs.copy()
coordinates = []

bg = pygame.image.load("Map.png")  # load background img
bg = pygame.transform.scale(bg, winsize)


#Collision check and line intersect
def ccw(a,b,c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

def check_intersect(node_a, node_b, obs):
    A = (node_a.x, node_a.y)
    B = (node_b.x, node_b.y)
    for o in obs:
        obs = (o[0], o[1], o[0] + o[2], o[1] + o[3])
        C1 = (obs[0], obs[1])
        D1 = (obs[0], obs[3])
        C2 = (obs[0], obs[1])
        D2 = (obs[2], obs[1])
        C3 = (obs[2], obs[3])
        D3 = (obs[2], obs[1])
        C4 = (obs[2], obs[3])
        D4 = (obs[0], obs[3])
        inst1 = ccw(A, C1, D1) != ccw(B, C1, D1) and ccw(A, B, C1) != ccw(A, B, D1)
        inst2 = ccw(A, C2, D2) != ccw(B, C2, D2) and ccw(A, B, C2) != ccw(A, B, D2)
        inst3 = ccw(A, C3, D3) != ccw(B, C3, D3) and ccw(A, B, C3) != ccw(A, B, D3)
        inst4 = ccw(A, C4, D4) != ccw(B, C4, D4) and ccw(A, B, C4) != ccw(A, B, D4)
        if inst1 == False and inst2 == False and inst3 == False and inst4 == False:
            continue
        else:
            return False
    return True

def check_intersect_points(x, y, a, b, obs):
    A = (x, y)
    B = (a, b)
    for o in obs:
        obs = (o[0], o[1], o[0] + o[2], o[1] + o[3])
        C1 = (obs[0], obs[1])
        D1 = (obs[0], obs[3])
        C2 = (obs[0], obs[1])
        D2 = (obs[2], obs[1])
        C3 = (obs[2], obs[3])
        D3 = (obs[2], obs[1])
        C4 = (obs[2], obs[3])
        D4 = (obs[0], obs[3])
        inst1 = ccw(A, C1, D1) != ccw(B, C1, D1) and ccw(A, B, C1) != ccw(A, B, D1)
        inst2 = ccw(A, C2, D2) != ccw(B, C2, D2) and ccw(A, B, C2) != ccw(A, B, D2)
        inst3 = ccw(A, C3, D3) != ccw(B, C3, D3) and ccw(A, B, C3) != ccw(A, B, D3)
        inst4 = ccw(A, C4, D4) != ccw(B, C4, D4) and ccw(A, B, C4) != ccw(A, B, D4)
        if inst1 == False and inst2 == False and inst3 == False and inst4 == False:
            continue
        else:
            return False
    return True

#Calculate Eucledian distance between two points in the environment
def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2)<epsilon:
        return p2
    else:
        theta = atan2(p2[1]-p1[1], p2[0]-p1[0])
        return p1[0]+epsilon*cos(theta), p1[1]+epsilon*sin(theta)

def choose_parent(nn, newnode, nodes):
    for p in nodes:
        if check_intersect(p,newnode,obs) and dist([p.x,p.y], [newnode.x, newnode.y]) < radius and p.cost +dist([p.x,p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x,nn.y],[newnode.x, newnode.y]):
            nn = p
    newnode.cost = nn.cost + dist([nn.x,nn.y], [newnode.x, newnode.y])
    newnode.parent = nn
    return newnode, nn

def reWire(nodes, newnode, pygame, screen):
    white = 255, 240, 200
    black = 20, 20, 40

    for i in range(len(nodes)):
        p = nodes[i]
        if check_intersect(p, newnode, obs) and \
                p != newnode.parent and dist([p.x, p.y], [newnode.x, newnode.y]) < radius and \
                newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y]) < p.cost:

            pygame.draw.line(screen, white, [p.x, p.y], [p.parent.x, p.parent.y])
            p.parent = newnode
            p.cost = newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y])
            nodes[i] = p
            pygame.draw.line(screen, black, [p.x, p.y], [newnode.x, newnode.y])
        return nodes

def drawSolutionPath(start, goal, nodes, pygame, screen, coordinates):
    pink = 200, 20, 240
    nn = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
            coordinates.append((nn.x, nn.y))
    while nn != start:
        pygame.draw.line(screen, pink, [nn.x, nn.y], [nn.parent.x, nn.parent.y], 5)
        nn = nn.parent

def find_optimal_path_from_nodes(start, goal, nodes, screen):
    optimal_path = []
    black = 0, 0, 0
    nn = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
            optimal_path.append(nn)
    while nn != start:
        pygame.draw.line(screen, black, [nn.x, nn.y], [nn.parent.x, nn.parent.y], 5)
        nn = nn.parent
    return optimal_path

def find_min_cost_path(start, goal, nodes, screen):
    opt_path = find_optimal_path_from_nodes(start, goal, nodes, screen)
    if len(opt_path) == 0:
        cost = 0
    else:
        cost = opt_path[-1].cost
    return cost, opt_path

class Cost:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord

class Node:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord=0, ycoord=0):
        self.x = xcoord
        self.y = ycoord
        self.cost = 0

    def rand_node_informed(self, screen, current_optimal_nodes):
        """ Returns a random node between boundaries set by informed sampling """
        x_values = [node.x for node in current_optimal_nodes]
        y_values = [node.y for node in current_optimal_nodes]

        y_min = round(min(y_values)) - 30
        y_max = round(max(y_values)) + 30
        x_min = round(min(x_values)) - 30
        x_max = round(max(x_values)) + 30

        self.x = random.random() * winsize[0]  # initial random guess
        self.y = random.random() * winsize[1]

        region = pygame.Rect(x_min, y_min, (x_max - x_min), (y_max - y_min))

        pygame.draw.rect(screen, color=(255, 0, 200), rect=region, width=2)

        while not region.collidepoint((self.x, self.y)):
            self.x = random.random() * winsize[0]
            self.y = random.random() * winsize[1]

        return self

def main():
    # initialize and prepare screen
    coordinates = []
    pygame.init()

    screen = pygame.display.set_mode(winsize)
    screen.blit(bg, (0, 0), special_flags=pygame.BLEND_ALPHA_SDL2)
    pygame.draw.circle(screen, (0, 0, 0), Start, 5, 10)
    pygame.draw.circle(screen, (0, 0, 0), Goal, 20, 3)
    black = 20, 20, 40

    # nodes
    nodes = [Node(Start[0], Start[1])]
    start = nodes[0]
    goal = Node(Goal[0], Goal[1])

    costs = {}
    i = 0
    near_goal_index = numnodes
    near_goal = False  # becomes true the first time we are near the goal
    calculating = True
    while calculating:
        i += 1
        if (i == numnodes) or (i == near_goal_index + 1000):  # 1000 extra nodes for optimization
            calculating = False

            # Informed sampling after we are near the goal
        if near_goal:
            rand = Node()
        else:
            rand = Node(random.random() * Map_dim[0], random.random() * Map_dim[1])

        nn = nodes[0]
        for p in nodes:
            if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn.x, nn.y], [rand.x, rand.y]):
                nn = p
        interpolatedNode = step_from_to([nn.x, nn.y], [rand.x, rand.y])

        newnode = Node(interpolatedNode[0], interpolatedNode[1])

        if check_intersect(nn, rand, obs):
            [newnode, nn] = choose_parent(nn, newnode, nodes)
            nodes.append(newnode)
            pygame.draw.line(screen, black, [nn.x, nn.y], [newnode.x, newnode.y])

            if near_goal:
                cost, opt_path = find_min_cost_path(start, goal, nodes, screen)
                costs[i] = cost

            nodes = reWire(nodes, newnode, pygame, screen)
            if round(nodes[-1].x) in [x for x in range(goal.x - 20, goal.x + 20)]:
                if round(nodes[-1].y) in [y for y in range(goal.y - 20, goal.y + 20)]:
                    if not near_goal:  # only the first time we are near the goal
                        print(f"We are near goal at index {i}")
                        near_goal_index = i  # index where tree found the goal for the first time
                        current_optimal_path = find_optimal_path_from_nodes(start, goal, nodes, screen)
                        near_goal = True

            pygame.display.update()

            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving because you requested it.")

    drawSolutionPath(start, goal, nodes, pygame, screen, coordinates)

    pygame.display.update()
    return near_goal, screen, coordinates


if __name__ == '__main__':
    running = True

    success, screen, result = main()

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
            #draw_obstacles_list(screen, obstacles_list)

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



















