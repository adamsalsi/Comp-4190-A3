__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import logging, threading, time
import numpy as np
import math
import random
from copy import copy
from random import randint
from pathplanning import PathPlanningProblem, Rectangle, Obstacle

import pygame
from pygame.locals import *

# Program Constants & Setup
##########################################
# For scaling the window of our simulation
SCALE = 7

# Simulation layout constants
MAX_ITERATIONS = 3000
MAX_FIELD_WIDTH = 100 * SCALE
MAX_FIELD_HEIGHT = 100 * SCALE
MIN_OBSTACLE_WIDTH = 10 * SCALE
MIN_OBSTACLE_HEIGHT = 10 * SCALE
MAX_OBSTACLE_WIDTH = 50 * SCALE
MAX_OBSTACLE_HEIGHT = 50 * SCALE
MAX_PULL = 100 * SCALE
STEP_LENGTH = 5 * SCALE
PATH_DD = 1 * SCALE

# Colour Constants
WHITE = 255, 240, 200
RED = 220, 20, 60
GREEN = 202, 255, 112

# Thread 1 Colours
BLACK = 20, 20, 40
BLUE = 0, 191, 255

# Thread 2 Colours
BROWN = 138, 51, 36
PURPLE = 178, 58, 238

# Threads
thread1 = None
thread2 = None

all_nodes = []
all_nodes2 = []

solutionFound = False

# Init playing field
pygame.init()
screen = pygame.display.set_mode([100*SCALE, 100*SCALE])
pygame.display.set_caption('Comp 4190 RRT Implementation')
screen.fill(WHITE)

# Time to find a solution
timeTaken = 0


# The primary node class
class Node:
    xList = []
    yList = []

    def __init__(self, newX, newY):
        # Node coordinates on playing field
        self.x = newX
        self.y = newY

        # Store random points of playing field
        self.xList = []
        self.yList = []

        # The node from which this node was branched, important for storing path
        self.parent = None

    def __str__(self):
        return " X: " + str(self.x) + " Y: " + str(self.y)


### Helper functions

# Simple method to find distance between two of our nodes
def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) * (node1.x - node2.x) + (node1.y - node2.y) * (node1.y - node2.y))


def steer(node1, node2):
    if distance(node1, node2) < PATH_DD:
        return node2
    else:
        theta = math.atan2(node2.y - node1.y, node2.x - node1.x)
        return Node(node1.x + PATH_DD * math.cos(theta), node1.y + PATH_DD * math.sin(theta))

# We find the nearest neighbour node, allowing us to reach goal sooner
def nearestNode(all_nodes, currNode):
    # Distances, from our tree all_nodes
    distances = [
                 (node.x - currNode.x) * (node.x - currNode.x)
               + (node.y - currNode.y) * (node.y - currNode.y)
               for node in all_nodes
                ]
    return all_nodes[distances.index(min(distances))]

# Exit the game & program
def checkExitSimulation():
    for e in pygame.event.get():
        if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
            return True
    return False
# Handle logic for goal
def checkGoal(newnode, goals, check_nodes, cost, numGoalPaths, goalColour, thread):
    atGoal = False
    goalDX = (newnode.x - goals[0])
    goalDY = (newnode.y - goals[1])

    # If we link threads, we link from this node ;)
    linkNode = None

    goalDistance = math.hypot(goalDX, goalDY)
    mergedPaths = False
    if thread is 1:
        global all_nodes2
        for nodes in all_nodes2:
            pathDX = (newnode.x - nodes.x)
            pathDY = (newnode.y - nodes.y)
            pathDistance = math.hypot(pathDX, pathDY)

            if pathDistance <= STEP_LENGTH:
                linkNode = nodes
                atGoal = True
                mergedPaths = True
    elif thread is 2:
        global all_nodes
        for nodes in all_nodes:
            pathDX = (newnode.x - nodes.x)
            pathDY = (newnode.y - nodes.y)
            pathDistance = math.hypot(pathDX, pathDY)

            if pathDistance <= STEP_LENGTH:
                linkNode = nodes
                atGoal = True
                mergedPaths = True

    if goalDistance <= STEP_LENGTH:
        atGoal = True

    if atGoal:
        numGoalPaths += 1
        newCost = 0

        # IF we used the other thread for a solution
        if mergedPaths is True:
            print("We found the goal by the powers of dual RRT, linking chains :) ")
            # print old thread's path
            currNode = linkNode

            # Draw connection
            pygame.draw.line(screen, goalColour, [currNode.x, currNode.y], [check_nodes[len(check_nodes) - 1].x, check_nodes[len(check_nodes) - 1].y])
            pygame.display.update()

            # Draw remainder
            while currNode.parent is not None:
                pygame.draw.line(screen, goalColour, [currNode.x, currNode.y], [currNode.parent.x, currNode.parent.y])
                pygame.display.update()
                newCost += distance(currNode, currNode.parent)
                currNode = currNode.parent

        # Print curr thread's path
        if mergedPaths is False and len(check_nodes) > 0:
            currNode = check_nodes[len(check_nodes) - 1]
            pygame.draw.line(screen, goalColour, [goals[0], goals[1]], [currNode.x, currNode.y])
            pygame.display.update()
            newCost += distance(Node(goals[0], goals[1]), currNode)
        elif mergedPaths is False:
            currNode = None
        while currNode.parent is not None:
            pygame.draw.line(screen, goalColour, [currNode.x, currNode.y], [currNode.parent.x, currNode.parent.y])
            pygame.display.update()
            newCost += distance(currNode, currNode.parent)
            currNode = currNode.parent

        # If we beat the old cost, update
        if cost is None or newCost < cost:
            cost = newCost

    return atGoal, cost, numGoalPaths

# Takes an initial point, the domain (with obstacles & goal), and the max iterations to find goal as params
def ExploreDomain(domain, initial, steps, goals, name):
    logging.info("Thread %s: starting", name)
    startTime = time.process_time()

    global solutionFound

    if name.__eq__("Thread 1"):
            pathColour = BLACK
            goalColour = BLUE
            thread = 1
            global all_nodes
    else:
            pathColour = BROWN
            goalColour = PURPLE
            thread = 2
            global all_nodes2

    cost = None
    foundGoal = False
    numGoalPaths = 0
    pos = np.array(initial)

    # Create the first node, this is where our implementation starts to deviate from sample
    initialNode = Node(initial[0], initial[1])
    if thread is 1:
        all_nodes = [initialNode]
    else:
        all_nodes2 = [initialNode]
    print("Our initial Node on the graph is : ", initialNode)

    # Init random path
    randomPath = [[initialNode.x, initialNode.y]]

    # TEST CODE
    for i in range(steps):
        if solutionFound is not True:
            # Pick a spot on the graph, proceed
            rand = Node(random.uniform(0, MAX_FIELD_WIDTH), random.uniform(0, MAX_FIELD_HEIGHT))

            # Init distance, closestNode = startingNode
            my_nodes = []
            if thread is 1:
                closestNode = all_nodes[0]
                my_nodes = all_nodes
            else:
                closestNode = all_nodes2[0]
                my_nodes = all_nodes2
            d = 0

            # Iterate to max cycles (Can be changed to give our algorithm more time to find a solution
            for theNodes in my_nodes:
                # Find closest node to random point
                if distance(theNodes, rand) < distance(closestNode, rand):
                    closestNode = theNodes
                    d = math.hypot(rand.x - closestNode.x, rand.y - closestNode.y)
            newnode = steer(closestNode, rand)

            # Call nearest node function to find our neighbour
            neighbourNode = nearestNode(my_nodes, newnode)

            # Store the path inside each node, later we can fetch for goal
            if d <= PATH_DD:
                r = Rectangle(rand.x, rand.y, 1*SCALE, 1*SCALE)
                # If no violation, add the random point to the list of coordinates in our current node
                if not domain.CheckOverlap(r):
                    newnode.xList.append(rand.x)
                    newnode.yList.append(rand.y)

            # Our current node's parent is simply its neighbour
            newnode.parent = neighbourNode

            # Check for obstacles
            r = Rectangle(newnode.x, newnode.y, 1*SCALE, 1*SCALE)
            if not domain.CheckOverlap(r):
                my_nodes.append(newnode)

            # Draw the current connection made to the simulation
            pygame.draw.line(screen, pathColour, [closestNode.x, closestNode.y], [newnode.x, newnode.y])
            pygame.display.update()

            # Check for goal and show goal path if found
            atGoal, cost, numGoalPaths = checkGoal(newnode, goals, my_nodes, cost, numGoalPaths, goalColour, thread)

            #Update if first time hitting goal
            if foundGoal is False and atGoal is True:
                foundGoal = True
                solutionFound = True
                print ("Found goal")

            # Optional, if we wish to end program once any goal path is found (good for dual rrt)
            if atGoal:
                print(name, "\n")
                print("Solution found, took the following length of time to complete : ", (time.process_time() - startTime), "s")
                return foundGoal

            # Check if user wishes to leave, IE ESCAPE
            endFlag = checkExitSimulation()
            if endFlag is True:
                print("Leaving simulation, sorry to see you go")
                exit(1)
        else:
            exit(1)
    # If we are out of the loop and still no goal, notify and return
    if foundGoal is False:
        print("Goal not found, try increasing max iterations")
    return foundGoal

def main(argv=None):
    if (argv == None):
        argv = sys.argv[1:]

    # Specify use of global threads
    global thread1
    global thread2

    # Random num obstacles (Could be 1-5 OBSTACLES CAN OVERLAP)
    num_obstacles = randint(1, 5)

    # Dimensions of grid, num objects, max dimensions of objects
    pp = PathPlanningProblem(MAX_FIELD_WIDTH, MAX_FIELD_HEIGHT, num_obstacles, MAX_OBSTACLE_WIDTH, MAX_OBSTACLE_HEIGHT, MIN_OBSTACLE_WIDTH, MIN_OBSTACLE_HEIGHT)

    # Call problem creator (SEE PART 2 Path Planning Simulation Domain)
    initial, goals = pp.CreateProblemInstance()

    # Plot the starting point
    pygame.draw.rect(screen, RED, (initial[0], initial[1], 1*SCALE, 1*SCALE))

    # Plot the obstacles
    for start in pp.obstacles:
        pygame.draw.rect(screen, BLACK, (start.x, start.y, start.width, start.height))

    # Plot the goal(s)
    for goal in goals:
        pygame.draw.rect(screen, GREEN, (goal[0], goal[1], 1*SCALE, 1*SCALE))

    # Extract single goal
    goal = [goals[0][0], goals[0][1]]

    # Run the dual RRT algorithm :)
    thread1 = threading.Thread(target=ExploreDomain, args=(pp, initial, MAX_ITERATIONS, goal, "Thread 1"))
    thread2 = threading.Thread(target=ExploreDomain, args=(pp, goal, MAX_ITERATIONS, initial, "Thread 2"))

    thread1.start()
    thread2.start()

    # Allow window to persist after execution, ESCAPE to quit
    while 1:
        pygame.display.update()
        if checkExitSimulation() is True:
            print("The marker has left the building...")
            exit(1)

if (__name__ == '__main__'):
    main()
