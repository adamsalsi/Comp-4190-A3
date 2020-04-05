__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
from pathplanning import PathPlanningProblem, Rectangle
import cellDecomposition
from cellDecomposition import CellDecomposition, QuadTreeDecomposition,BinarySpacePartitioning


class AStar:
    def __init__(self, decomp, pp, initial, goals, ax):
        self.decomp = decomp
        self.initial = initial
        self.goals = goals
        self.root = decomp.root
        self.cellList = []
        self.neighbor = []
        self.__transverse(self.root)
        self.startNode = None
        self.goalNode = None
        self.pp = pp
        self.__iniProblemInstance()
        self.ax=ax

        self.path=None

    # assign the initial and goal
    def __iniProblemInstance(self):
        # repeat until we found the node where points are in
        # there is chance we can not found due to quadtree square so small, and can not find the node
        # (float comparision loss precision)

        # while self.startNode is None or self.goalNode is None:
        # self.initial, self.goals = self.pp.CreateProblemInstance()
        self.startNode=self.__findNode(self.initial,self.root)
        self.goalNode=self.__findNode(self.goals[0],self.root)


    # transverse the tree to find all free cell
    def __transverse(self, node=None):
        if (node == None):
            node = self.root
        # add free node as empty cell for A*
        if (node[1] == 'free'):
            self.cellList.append(node)
        for c in node[2]:
            self.__transverse(c)

    # transverse the tree to find all free cell
    # do range search in quad tree find the neighbor
    # def __findNeighbor(self):
    #    for cell in self.neighbor:

    def __findNode(self, pt, tree):
        curr = tree
        if curr[0].InRect(pt):
            # the goal/or start can only be in mix/free tile
            if curr[1] == 'mixed':
                # child( quadrant ) of node
                for c in curr[2]:
                    rtn = self.__findNode(pt, c)
                    # if found 'free' quadrant, end
                    if rtn is not None:
                        return rtn
                    # continue search other child
            elif curr[1] == 'free':
                return curr
            else:
                return None
        return None

    #def __findNeighbor(self, rec1, ):

    def findPath(self):
        self.path=self.searchPath()
        # highlight path
        path=self.path
        if path is None:
            print("No path between")
        else:
            # reverse the path to have correcr order
            reversed_path = []
            while path is not None:
                reversed_path.append(path)
                path = path.pre
            reversed_path.reverse()
            print("Start: ", self.initial)
            for path in reversed_path:
                tile = path.node
                QuadTreeDecomposition.setColor(tile)
                print("Cost: ", path.g, " Cell Rectangle Path: ", path.node[0])
            print("Goal: ", self.goals)
            print(
                "Note: robot will first move to center of cell at start and out of center to goal at last cell, and move from center to center between cell")
        # ------------------------

    def searchPath(self):
        # to be check node
        goal = self.goalNode
        start = self.startNode
        h = AStar.heuristic(start, goal)
        # since we move from center of square
        rect=start[0]
        centerX = rect.x+rect.width/2
        centerY = rect.y+rect.height/2
        g = math.sqrt(math.pow(self.initial[0]-centerX, 2)+math.pow(self.initial[1]-centerY, 2))
        f = g+h
        curNode=Node(self.startNode, h, g, f, None)
        open=[curNode]
        # visited node
        closed=[]
        while len(open)>0:
            #print("Open", len(open), "Closed", len(closed))
            curr = open.pop(0)
            curNode=curr.node
            # we are in goal node?
            if curNode == goal:
                rect = goal[0]
                centerX = rect.x + rect.width / 2
                centerY = rect.y + rect.height / 2
                g = math.sqrt(math.pow(self.goals[0][0] - centerX, 2) + math.pow(self.goals[0][1] - centerY, 2))
                # we move to center first then to the actual point
                curr.f = curr.f+g
                print('Found Goal')
                return curr

            # add neighbor to search path
            neighbors=(self.decomp).get_neighborsAll(curNode)
            #print("NB ", len(neighbors), "------------- OF ", curNode[0])
            for nb in neighbors:
                #print(nb[0])
                # cell in the open list
                # node not in tobe visited list
                if Node.inList(nb, open) is None:
                    oldNode = Node.inList(nb, closed)
                    # heuristic cost next to goal
                    h = AStar.heuristic(nb, goal)
                    # cost to go to next cell
                    g = curr.g + AStar.heuristic(curNode, nb)
                    f = g+h
                    # visited node?
                    if oldNode is None:
                        insertNode = Node(nb, h, g, f, curr)
                        AStar.prioInsert(insertNode, open)
                    # # admissible guarantee  optimal path , no need to update
                    else:
                        # found a shorter path
                        if oldNode.f > f:
                            # update new value for shorter path
                            oldNode.g = g
                            oldNode.h = h
                            oldNode.f = f
                            # update path pointer
                            oldNode.node = curr
                        # not visited node
            closed.append(curr)
        return None

    @staticmethod
    def printList(list):
        print("[")
        for elm in list:
            print(elm.f,end=', ')
        print("]")

    @staticmethod
    def prioInsert(node, list):
        i = 0
        # when same priority always at beginning
        while i < len(list) and node > (list[i]):
            i = i+1
        list.insert(i, node)
        #AStar.printList(list)
        return None



    # use Euclidian distance, path will be admmisive, no overestimate, gurantee optimal pass
    @staticmethod
    def heuristic(start, end):
        # get the rectangle of node
        rec = start[0]
        # calculate
        x0 = rec.x+rec.width/2
        y0 = rec.y+rec.height/2
        rec = end[0]
        x1 = rec.x+rec.width/2
        y1 = rec.y+rec.height/2
        dis = math.sqrt(math.pow(x0-x1, 2)+math.pow(y0-y1, 2))
        return dis



class Node:
    def __init__(self,node, h,g, f, pre):
        self.node=node
        self.h=h
        self.g=g
        self.f=f
        # store path
        self.pre=pre

    #   # For x >= y
    def __gt__(self, other):
        """Override the default Unequal behavior"""
        return self.f > other.f

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.node == other.node
        else:
            return False

    @staticmethod
    def inList(item,list):
        if list is None:
            return None
        for elm in list:
            if item==elm.node:
                return elm
        return None



def main(argv=None):
    if (argv == None):
        argv = sys.argv[1:]


if (__name__ == '__main__'):
    main()
