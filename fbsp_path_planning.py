__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import aStart as ast
import sys
import matplotlib.pyplot as plt
import copy
from pathplanning import PathPlanningProblem
from cellDecomposition import QuadTreeDecomposition, BinarySpacePartitioning

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 100.0
    height = 100.0
    onum=5
    oheight=50
    oheight_min=10
    owidth=50
    owidth_min=10
    targeSize=2
    halfSize=targeSize/2


    pp = PathPlanningProblem( width, height, onum, owidth, oheight, owidth_min, oheight_min)
    #pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    initial, goals = pp.CreateProblemInstance()


    # goalN=QuadTreeDecomposition.get_neighborsAll(test.goalNode )
    # startN=QuadTreeDecomposition.get_neighborsAll(test.startNode )
    # for nb in startN:
    #     QuadTreeDecomposition.setColor(nb)
    # for nb in goalN:
    #     QuadTreeDecomposition.setColor(nb)


    fig = plt.figure()
    ax = fig.add_subplot(1,2,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    # initial = test.initial
    # goals = test.goals
    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch) )
    ip = plt.Rectangle((initial[0]-halfSize,initial[1]-halfSize), targeSize, targeSize, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0]-halfSize,g[1]-halfSize), targeSize, targeSize, facecolor='#00ff00')
        ax.add_patch(g)

    qtd = QuadTreeDecomposition(pp, 0.5)
    test = ast.AStar(qtd, pp, initial, goals, ax)
    test.findPath()
    qtd.Draw(ax)
    n = qtd.CountCells()

    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    ax = fig.add_subplot(1,2,2, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)


    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch))
    ip = plt.Rectangle((initial[0]-halfSize,initial[1]-halfSize), targeSize, targeSize, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0]-halfSize,g[1]-halfSize), targeSize, targeSize, facecolor='#00ff00')
        ax.add_patch(g)

    bsp = BinarySpacePartitioning(pp, 0.25)
    test = ast.AStar(bsp,pp, initial, goals,ax)
    test.findPath()

    bsp.Draw(ax)
    n = bsp.CountCells()
    ax.set_title('BSP Decomposition\n{0} cells'.format(n))

    plt.show()

if ( __name__ == '__main__' ):
    main()


