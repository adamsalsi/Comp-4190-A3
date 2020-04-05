__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import matplotlib.pyplot as plt
import math
from pathplanning import Rectangle

# cell decomposition parent class for quad and fbsp
class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', [], None, 0, 0]

    # drawing the cell
    def Draw(self, ax, node = None):
            if ( node == None ):
                node = self.root
            r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
            if ( node[1] == 'mixed' ):
                color = '#00ff80'
                if ( node[2] == [] ):
                    r.set_fill(True)
                    r.set_facecolor(color)
            elif ( node[1] == 'free' ):
                color = '#ffff00'
                if len(node)>=5 and node[5] == 1:
                    color = '#ff0000'
                r.set_fill(True)
                r.set_facecolor(color)
            elif ( node[1] == 'obstacle'):
                color = '#5050ff'
                r.set_fill(True)
                r.set_facecolor(color)
            else:
                print("Error: don't know how to draw cell of type", node[1])
            #print('Draw node', node)
            ax.add_patch(r)
            for c in node[2]:
                self.Draw(ax, c)

    def CountCells(self, node = None ):
        if ( node is None ):
            node = self.root
        sum = 0
        if ( node[2] != [] ):
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum

    # set special color of region
    @staticmethod
    def setColor(node):
        node[5] = 1

    @staticmethod
    def clearColor(node):
        node[5] = 0

    # def get_neighborsAll(self, node):
    #     raise NotImplementedError("Subclass must implement abstract method")

class QuadTreeDecomposition(CellDecomposition):
    # define class variable for location
    SW = 0
    SE = 1
    NW = 2
    NE = 3
    # define direction of neighbor
    N = 7
    S = 8
    W = 9
    E =10

    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        # store root region
        self.root = self.Decompose(self.root)

    def Decompose(self, parent):
        cell = 'free'
        r = parent[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break
        # break down only when it s mixed
        if cell == 'mixed':
            if (rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize):
                # node structure [rec, name, child, parent]
                soth_west = [Rectangle(rx, ry, rwidth/2.0, rheight/2.0), 'unknown', [], parent, self.SW, 0 ]
                q_soth_west = self.Decompose( soth_west )
                south_east = [Rectangle(rx + rwidth/2.0, ry, rwidth/2.0, rheight/2.0), 'unknown', [], parent, self.SE, 0]
                q_south_east = self.Decompose( south_east )
                north_west = [Rectangle(rx, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [], parent, self.NW, 0]
                q_north_west = self.Decompose( north_west )
                north_east = [Rectangle(rx + rwidth/2.0, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [], parent, self.NE, 0]
                q_north_east = self.Decompose( north_east )
                children = [q_soth_west, q_south_east, q_north_west, q_north_east]
                parent[2] = children
            else:
                cell = 'obstacle'
        parent[1] = cell
        return parent


    # get neighbor of greater or equal size
    @staticmethod
    def get_neighbor_of_greater_or_equal_size(node, direction):
        rtn=None
        # none node or parent is none
        if node is None or node[3] is None:  # Reached root?
            return None
        # find neighbor of north
        if direction == QuadTreeDecomposition.N:
            parent=node[3]
            parentChildren=parent[2]
            if node[4] == QuadTreeDecomposition.SW:  # Is node SW child?
                # get NW neighbor
                child=parentChildren[QuadTreeDecomposition.NW]
                return child
            elif node[4] == QuadTreeDecomposition.SE:  # Is node SE child?
                # get NE child
                child=parentChildren[QuadTreeDecomposition.NE]
                return child
            #  node is a north child, node is NW or NE child
            rtn = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node[3], direction)

            # only return the node with free space neighbor
            # none-mixed node is leaf node
            # the node is greater than the target node, terminate when it s 'free' or 'obstacle'
            if rtn is None or rtn[1]!='mixed':
                return rtn
            # it s a mixed type and bigger grid
            elif rtn[1]=='mixed':
                children=rtn[2]
                if node[4] == QuadTreeDecomposition.NW:  # Is 'self' NW child?
                    rtn=children[QuadTreeDecomposition.SW]
                elif node[4]==QuadTreeDecomposition.NE:   # Is 'self' NE child
                    rtn=children[QuadTreeDecomposition.SE]
        elif direction == QuadTreeDecomposition.S:
            parent=node[3]
            parentChildren=parent[2]
            if node[4] == QuadTreeDecomposition.NW:  # Is node NW child?
                # get SW neighbor
                child=parentChildren[QuadTreeDecomposition.SW]
                return child
            elif node[4] == QuadTreeDecomposition.NE:  # Is node NE child?
                # get SE child
                child=parentChildren[QuadTreeDecomposition.SE]
                return child
            #  node is a south child
            rtn = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node[3], direction)

            # only return the node with free space neighbor
            # none-mixed node is leaf node
            # the node is greater than the target node, terminate when it s 'free' or 'obstacle'
            if rtn is None or rtn[1]!='mixed':
                return rtn
            # it s a mixed type and bigger grid
            elif rtn[1]=='mixed':
                children=rtn[2]
                if node[4] == QuadTreeDecomposition.SW:  # Is 'self' SW child?
                    rtn=children[QuadTreeDecomposition.NW]
                elif node[4]==QuadTreeDecomposition.SE:   # Is 'self' SE child
                    rtn=children[QuadTreeDecomposition.NE]
        elif direction == QuadTreeDecomposition.E:
            parent=node[3]
            parentChildren=parent[2]
            if node[4] == QuadTreeDecomposition.NW:  # Is node NW child?
                # get NE neighbor
                child=parentChildren[QuadTreeDecomposition.NE]
                return child
            elif node[4] == QuadTreeDecomposition.SW:  # Is node SW child?
                # get SE child
                child=parentChildren[QuadTreeDecomposition.SE]
                return child
            #  node is a east child
            rtn = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node[3], direction)

            # only return the node with free space neighbor
            # none-mixed node is leaf node
            # the node is greater than the target node, terminate when it s 'free' or 'obstacle'
            if rtn is None or rtn[1]!='mixed':
                return rtn
            # it s a mixed type and bigger grid
            elif rtn[1]=='mixed':
                children=rtn[2]
                if node[4] == QuadTreeDecomposition.SE:  # Is 'self' SE child?
                    rtn=children[QuadTreeDecomposition.SW]
                elif node[4]==QuadTreeDecomposition.NE:   # Is 'self' NE child
                    rtn=children[QuadTreeDecomposition.NW]
        elif direction == QuadTreeDecomposition.W:
            parent=node[3]
            parentChildren=parent[2]
            if node[4] == QuadTreeDecomposition.NE:  # Is node NE child?
                # get NW neighbor
                child=parentChildren[QuadTreeDecomposition.NW]
                return child
            elif node[4] == QuadTreeDecomposition.SE:  # Is node SE child?
                # get SW child
                child=parentChildren[QuadTreeDecomposition.SW]
                return child
            #  node is a east child
            rtn = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node[3], direction)

            # only return the node with free space neighbor
            # none-mixed node is leaf node
            # the node is greater than the target node, terminate when it s 'free' or 'obstacle'
            if rtn is None or rtn[1]!='mixed':
                return rtn
            # it s a mixed type and bigger grid
            elif rtn[1]=='mixed':
                children=rtn[2]
                if node[4] == QuadTreeDecomposition.SW:  # Is 'self' SW child?
                    rtn=children[QuadTreeDecomposition.SE]
                elif node[4]==QuadTreeDecomposition.NW:   # Is 'self' NW child
                    rtn=children[QuadTreeDecomposition.NE]
        return rtn

    @staticmethod
    def find_neighbors_of_smaller_size(neighbor, direction):
        candidates = [] if neighbor is None else [neighbor]
        neighbors = []

        if direction == QuadTreeDecomposition.N:
            while len(candidates) > 0:
                curr=candidates[0]
                if curr[1] == 'free':
                    neighbors.append(curr)
                elif curr[1]== 'mixed':
                    curr=curr[2]
                    candidates.append(curr[QuadTreeDecomposition.SW])
                    candidates.append(curr[QuadTreeDecomposition.SE])
                candidates.remove(candidates[0])
        elif direction == QuadTreeDecomposition.S:
            while len(candidates) > 0:
                curr = candidates[0]
                if curr[1] == 'free':
                    neighbors.append(curr)
                elif curr[1] == 'mixed':
                    curr = curr[2]
                    candidates.append(curr[QuadTreeDecomposition.NW])
                    candidates.append(curr[QuadTreeDecomposition.NE])
                candidates.remove(candidates[0])
        elif direction == QuadTreeDecomposition.E:
            while len(candidates) > 0:
                curr=candidates[0]
                if curr[1] == 'free':
                    neighbors.append(curr)
                elif curr[1]== 'mixed':
                    curr=curr[2]
                    candidates.append(curr[QuadTreeDecomposition.NW])
                    candidates.append(curr[QuadTreeDecomposition.SW])
                candidates.remove(candidates[0])
        elif direction == QuadTreeDecomposition.W:
            while len(candidates) > 0:
                curr=candidates[0]
                if curr[1] == 'free':
                    neighbors.append(curr)
                elif curr[1]== 'mixed':
                    curr=curr[2]
                    candidates.append(curr[QuadTreeDecomposition.NE])
                    candidates.append(curr[QuadTreeDecomposition.SE])
                candidates.remove(candidates[0])
        return neighbors

    @staticmethod
    def get_neighbors( node, direction):
      neighbor = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node, direction)
      neighbors = QuadTreeDecomposition.find_neighbors_of_smaller_size( neighbor, direction)
      return neighbors


    def get_neighborsAll(self, node):
        neighbors=[]
        neighbor = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node, QuadTreeDecomposition.N)
        neighbors.extend(QuadTreeDecomposition.find_neighbors_of_smaller_size(neighbor, QuadTreeDecomposition.N))
        neighbor = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node, QuadTreeDecomposition.S)
        neighbors.extend(QuadTreeDecomposition.find_neighbors_of_smaller_size(neighbor, QuadTreeDecomposition.S))
        neighbor = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node, QuadTreeDecomposition.E)
        neighbors.extend(QuadTreeDecomposition.find_neighbors_of_smaller_size(neighbor, QuadTreeDecomposition.E))
        neighbor = QuadTreeDecomposition.get_neighbor_of_greater_or_equal_size(node, QuadTreeDecomposition.W)
        neighbors.extend(QuadTreeDecomposition.find_neighbors_of_smaller_size(neighbor, QuadTreeDecomposition.W))
        return neighbors

# cell decomposition using binary space partition using entropy heuristic
class BinarySpacePartitioning(CellDecomposition):
    def __init__(self, domain, minimumSize ):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Entropy(self, p):
        e = 0.0
        if ( ( p > 0 ) and ( p < 1.0 ) ):
            e = -p * math.log(p,2) - (1-p) * math.log(1-p,2)
        return e

    def CalcEntropy(self, rect):
        area = rect.width * rect.height
        a = 0.0
        rectList=[]
        rectList.extend(self.domain.obstacles)
        a = self.CalMultiOverLapArea(rect,rectList, 1, 0)
        p = a / area
        return self.Entropy(p)

    # multiple rectangle overlap are
    def CalMultiOverLapArea(self, rect,rectList, sign, index):
        area = 0
        for i in range(index, len(rectList)):
            # do overlap?
            target=rectList[i]
            # check if the rect overlap
            if rect.CalculateOverlap(target) > 0:
                rectOverlap = rect.CaluculateOverlapRect(target)
                area = area+rectOverlap.CaluclateArea()
                # recursive call
                area = area+self.CalMultiOverLapArea(rectOverlap, rectList, (-1)*sign, i+1)
        return sign*area



    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height
        area = rwidth * rheight



        for o in self.domain.obstacles:
            # if fill the rect guarantee to be obstacle
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            # still can be obstacle when overlap
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break

        cal=self.CalMultiOverLapArea(r, self.domain.obstacles, 1, 0)
        if cal >= rwidth * rheight:
             cell = 'obstacle'
        # elif cal>0:
        #     cell='mixed'

        if ( cell == 'mixed'):
            entropy = self.CalcEntropy(r)
            igH = 0.0
            # horizontal split
            hSplitTop = None
            hSplitBottom = None
            # vertical split
            vSplitLeft = None
            vSplitRight = None
            # split horizontally into 2 pieces
            if ( r.height / 2.0 > self.minimumSize):
                hSplitTop = Rectangle(rx, ry + rheight/2.0, rwidth, rheight/2.0)
                entHSplitTop = self.CalcEntropy(hSplitTop)
                hSplitBottom = Rectangle(rx, ry, rwidth, rheight/2.0)
                entHSplitBottom = self.CalcEntropy( hSplitBottom )

                # information gain horizontal
                igH = entropy - ( r.width * r.height / 2.0 ) / area * entHSplitTop \
                      - ( r.width * r.height / 2.0 ) / area * entHSplitBottom
            igV = 0.0
            # split vertically into 2 pieces
            if ( r.width / 2.0 > self.minimumSize ):
                vSplitLeft = Rectangle(rx, ry, rwidth/2.0, rheight )
                entVSplitLeft = self.CalcEntropy( vSplitLeft )
                vSplitRight = Rectangle( rx + rwidth/2.0, ry, rwidth/2.0, rheight)
                entVSplitRight = self.CalcEntropy( vSplitRight)

                # information gain vertical
                igV = entropy - ( r.width/2.0 * r.height ) / area * entVSplitLeft \
                      - ( r.width/2.0 * r.height ) / area * entVSplitRight
            # decomposition
            children = []
            if ( igH > igV ):
                if ( igH > 0.0 ):
                    if ( hSplitTop is not None ) and ( hSplitBottom is not None ):
                        childTop = [ hSplitTop, 'unknown', [], node, -1, 0]
                        childBottom = [hSplitBottom, 'unknown', [], node, -1, 0]
                        children = [ childTop, childBottom]
            else:
                if ( igV > 0.0 ):
                    if ( vSplitLeft is not None ) and ( vSplitRight is not None ):
                        childLeft = [vSplitLeft, 'unknown', [], node, -1, 0]
                        childRight = [ vSplitRight, 'unknown', [], node, -1, 0 ]
                        children = [ childLeft, childRight ]
            for c in children:
                self.Decompose(c)
            node[2] = children
        node[1] = cell
        return node

    # find the left neighbor
    def findLeftNb(self, rect, node):
        testRect=self.nbLeftTest(rect)
        if (node == None):
            node = self.root
        nb=self.__findOverlapNb(testRect,node)
        return nb

    # find the right neighbor
    def findRightNb(self, rect, node):
        testRect=self.nbRightTest(rect)
        if (node == None):
            node = self.root
        nb=self.__findOverlapNb(testRect,node)
        return nb

    # find the top neighbor
    def findTopNb(self, rect, node):
        testRect=self.nbTopTest(rect)
        if (node == None):
            node = self.root
        nb=self.__findOverlapNb(testRect,node)
        return nb

    # find the bottom neighbor
    def findBotNb(self, rect, node):
        testRect=self.nbBotTest(rect)
        if (node == None):
            node = self.root
        nb=self.__findOverlapNb(testRect,node)
        return nb

    # find the rect such that overlap with test rect
    def __findOverlapNb(self, testRect, node):
        nb = []
        if node[1] == 'obstacle':
            return nb
        else:
            overlap = testRect.CalculateOverlap(node[0])
            if overlap > 0:
                if (node[1] == 'mixed'):
                    for c in node[2]:
                        neighbors=self.__findOverlapNb(testRect, c)
                        nb.extend(neighbors)
                elif (node[1] == 'free'):
                    nb.append(node)
        return nb

    # find the bounding box on left side of rect
    def nbLeftTest(self,rect):
        x = rect.x-self.minimumSize
        width = self.minimumSize
        y = rect.y
        height = rect.height
        test = Rectangle(x, y, width, height)
        return test

    # find the bounding box on right side of rect
    def nbRightTest(self,rect):
        x=rect.x+rect.width
        width=self.minimumSize
        y=rect.y
        height=rect.height
        test=Rectangle(x,y,width,height)
        return test

    # find the bounding box on bottom side of rect
    def nbBotTest(self,rect):
        x=rect.x
        height=self.minimumSize
        y=rect.y-self.minimumSize
        width=rect.width
        test=Rectangle(x,y,width,height)
        return test

    # find the bounding box on top side of rect
    def nbTopTest(self,rect):
        x=rect.x
        height=self.minimumSize
        y=rect.y+rect.height
        width=rect.width
        test=Rectangle(x,y,width,height)
        return test

    # find the neighbor of top,bottom, left, right
    def get_neighborsAll(self,node):
        nb=[]
        node=node[0]
        nbTemp=self.findLeftNb(node, None)
        nb.extend(nbTemp)
        nbTemp=self.findRightNb(node, None)
        nb.extend(nbTemp)
        nbTemp=self.findTopNb(node, None)
        nb.extend(nbTemp)
        nbTemp=self.findBotNb(node, None,)
        nb.extend(nbTemp)
        return nb