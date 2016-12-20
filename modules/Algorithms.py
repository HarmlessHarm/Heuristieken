# -*- coding: utf-8 -*-
import numpy as np
from random import shuffle
import sys
from Objects import *
from Visualizer import *
import copy
import pprint
import datetime
import time
import random

class AStar(object):

    """Initialize the A* algorithm with a board and net for which to plan a path, optional is a bias parameter (either 'vertical' or 'lateral')

    Args:
        board (:obj: Board): The board on which to plan a path
        net (:obj: Net): The net for which to plan a path
        bias (:obj:'str', optional): A flag indicating which distance bias to use.
    """

    def __init__(self, board, net, bias='vertical', multiple=False):
        super(AStar, self).__init__()
        self.board = board
        self.net = net
        self.bias = bias
        self.multiple = multiple

    def createPath(self):
        """Run the algorithm to create a path

        Returns:
            list: empty if no path exists, ordered set of (x,y,z)-coordinates if there is.
        """
        (x, y, z) = self.board.getDimensions()
        start = self.board.gates[self.net.start_gate].getCoordinates()
        (x_start, y_start, z_start) = start
        goal = self.board.gates[self.net.end_gate].getCoordinates()
        (x_end, y_end, z_end) = goal
        # For each node, whether it has been evaluated
        closedSet = np.zeros((x, y, z), dtype=bool)

        # Set of discovered nodes that still need evaluation, where a node is a
        # tuple (x,y,z)
        openSet = [start]

        # Dictionary that for each node (a tuple x,y,z) archives which node it
        # can most easily be reached from
        cameFrom = {}

        # Array that contains g_scores for all nodes, the cost of getting from start to that node
        # default is -1, as placeholder for proper default infinity
        gScore = np.full((x, y, z), -1, dtype='int64')
        # Cost of going from start to start is 0
        gScore[x_start][y_start][z_start] = 0

        # array that contains f_scores for all nodes, the distance for getting to the goal node from start via that node
        # Default is -1, as placeholder for infinity
        fScore = np.full((x, y, z), -1, dtype='int64')
        fScore[x_start][y_start][
            z_start] = self.manhattanCostEstimate(start, goal)

        bestLen = sys.maxint

        while openSet != []:
            # Set currentNode to be the node in openset with the lowest fscore
            # (above -1)
            (cx, cy, cz) = openSet[0]
            for (x, y, z) in openSet:
                if 0 <= fScore[x][y][z] < fScore[cx][cy][cz]:
                    (cx, cy, cz) = (x, y, z)

            # if currentNode is adjacent to goal node, return the path to
            # currentNode
            if goal in self.board.getAllNeighbours(cx, cy, cz):
                cameFrom[goal] = (cx, cy, cz)
                path = self.reconstructPath(cameFrom, goal)
                return path

            openSet.remove((cx, cy, cz))
            closedSet[cx][cy][cz] = 1

            for (nx, ny, nz) in self.board.getOpenNeighbours(cx, cy, cz):
                if closedSet[nx][ny][nz]:
                    continue  # neighbour is already evaluated

                tentative_gscore = gScore[cx][cy][
                    cz] + self.distance((nx, ny, nz))
                if not (nx, ny, nz) in openSet:
                    openSet.append((nx, ny, nz))
                elif tentative_gscore >= gScore[nx][ny][nz] >= 0:
                    continue

                cameFrom[(nx, ny, nz)] = (cx, cy, cz)
                gScore[nx][ny][nz] = tentative_gscore
                fScore[nx][ny][nz] = gScore[nx][ny][nz] + \
                    self.manhattanCostEstimate((nx, ny, nz), goal)

        return []

    def distance(self, node):
        """Distance to node, based on its neighbours and the layer it is in, the bias in distance added is set when the objact is instantiated

        Args:
            node (tuple): a coordinate

        Returns:
            int: Distance to the node
        """
        (x, y, z) = node
        distance = 1
        # Vertical bias
        if self.bias == 'vertical':
            for (nx, ny, nz) in self.board.getAllNeighbours(x, y, z):
                if type(self.board.getElementAt(nx, ny, nz)) is Gate:
                    # should be just enough to make the path that leaves one
                    # space around a gate be cheaper than the path that doesn't
                    distance += 4
                elif type(self.board.getElementAt(nx, ny, nz)) is Net:
                    # Add one distance for every adjacent net, this should
                    # space things out a bit
                    distance += 3

                # Make higher paths more attractive
                if self.bias == 'vertical':
                    distance += pow(self.board.z_dim, 2) / (nz+1)
                # Make paths that avoid the center of the board more attractive
                elif self.bias == 'lateral':
                    xCenter = self.board.x_dim/2
                    yCenter = self.board.y_dim/2
                    distanceFromCenter = abs(nx-xCenter)+abs(ny-yCenter)
                    distance += min(xCenter, yCenter) / (distanceFromCenter+1)
                    distance += pow(self.board.z_dim, 2) / (nz+1)

        return distance

    # Very optimistic heuristic, it returns the manhattan distance between the
    # 2 nodes
    def manhattanCostEstimate(self, node1, node2):
        (x, y, z) = node1
        (x2, y2, z2) = node2
        return abs(x2-x)+abs(y2-y)+abs(z2-z)

    # Helper function to reconstruct the path
    def reconstructPath(self, cameFrom, currentNode):
        # pprint.pprint( cameFrom)
        if self.multiple:
            paths = []
            allPaths = self.returnMultiplePaths(cameFrom, [currentNode], paths)
            return allPaths
        else:
            path = [currentNode]
            while currentNode in cameFrom.keys():
                currentNode = cameFrom[currentNode]
                path.append(currentNode)
            return list(reversed(path))

    def returnMultiplePaths(self, cameFrom, path, paths):
        lastNode = path[-1]
        if lastNode in cameFrom.keys():
            for nextNode in cameFrom[lastNode]:
                new_path = path + [nextNode]
                paths = self.returnMultiplePaths(cameFrom, new_path, paths)
        else:
            path = list(reversed(path))
            paths += [path]
        return paths


class AStarAllPaths(object):

    """Initialize the A* algorithm with a board and net for which to plan a path, optional is a bias parameter (either 'vertical' or 'lateral')

    Args:
        board (:obj: Board): The board on which to plan a path
        net (:obj: Net): The net for which to plan a path
        bias (:obj:'str', optional): A flag indicating which distance bias to use.
    """

    def __init__(self, board, net):
        super(AStarAllPaths, self).__init__()
        self.board = board
        self.net = net

    def createPath(self):
        """Run the algorithm to create a path

        Returns:
            list: empty if no path exists, ordered set of (x,y,z)-coordinates if there is.
        """
        (x, y, z) = self.board.getDimensions()
        start = self.board.gates[self.net.start_gate].getCoordinates()
        (x_start, y_start, z_start) = start
        goal = self.board.gates[self.net.end_gate].getCoordinates()
        (x_end, y_end, z_end) = goal
        # For each node, whether it has been evaluated
        closedSet = np.zeros((x, y, z), dtype=bool)

        # Set of discovered nodes that still need evaluation, where a node is a
        # tuple (x,y,z)
        openSet = [start]

        # Dictionary that for each node (a tuple x,y,z) archives which node it
        # can most easily be reached from
        cameFrom = {}

        # Array that contains g_scores for all nodes, the cost of getting from start to that node
        # default is -1, as placeholder for proper default infinity
        gScore = np.full((x, y, z), -1, dtype='int64')
        # Cost of going from start to start is 0
        gScore[x_start][y_start][z_start] = 0

        # array that contains f_scores for all nodes, the distance for getting to the goal node from start via that node
        # Default is -1, as placeholder for infinity
        fScore = np.full((x, y, z), -1, dtype='int64')
        fScore[x_start][y_start][
            z_start] = self.manhattanCostEstimate(start, goal)

        bestLen = sys.maxint
        # print goal
        goalFound = False
        while openSet != []:
            # print openSet
            # print len(cameFrom.keys())
            # Set currentNode to be the node in openset with the lowest fscore
            # (above -1)
            (cx, cy, cz) = openSet[0]
            for (x, y, z) in openSet:
                if 0 <= fScore[x][y][z] < fScore[cx][cy][cz]:
                    (cx, cy, cz) = (x, y, z)

            # if currentNode is adjacent to goal node, return the path to
            # currentNode
            if goal in self.board.getAllNeighbours(cx, cy, cz):
                if goal in cameFrom.keys():
                    cameFrom[goal].append((cx, cy, cz))
                else:
                    cameFrom[goal] = [(cx, cy, cz)]
                # path = self.reconstructPath(cameFrom, goal)
                goalFound = True
                return self.reconstructPath(cameFrom, goal)
                # return path

            openSet.remove((cx, cy, cz))

            for (nx, ny, nz) in self.board.getOpenNeighbours(cx, cy, cz):
                if closedSet[nx][ny][nz]:
                    continue  # neighbour is already evaluated

                tentative_gscore = gScore[cx][cy][
                    cz] + self.distance((nx, ny, nz))
                if not (nx, ny, nz) in openSet and not goalFound:
                    openSet.append((nx, ny, nz))
                elif tentative_gscore > gScore[nx][ny][nz] >= 0:
                    continue

                if (nx, ny, nz) in cameFrom.keys():
                    cameFrom[(nx, ny, nz)].append((cx, cy, cz))
                else:
                    cameFrom[(nx, ny, nz)] = [(cx, cy, cz)]
                gScore[nx][ny][nz] = tentative_gscore
                fScore[nx][ny][nz] = gScore[nx][ny][nz] + \
                    self.manhattanCostEstimate((nx, ny, nz), goal)

            closedSet[cx][cy][cz] = 1
        return self.reconstructPath(cameFrom, goal)

    def distance(self, node):
        """Distance to node, based on its neighbours and the layer it is in, the bias in distance added is set when the objact is instantiated

        Args:
            node (tuple): a coordinate

        Returns:
            int: Distance to the node
        """
        (x, y, z) = node
        distance = 1
        # distance += z

        return distance

    # Very optimistic heuristic, it returns the manhattan distance between the
    # 2 nodes
    def manhattanCostEstimate(self, node1, node2):
        (x, y, z) = node1
        (x2, y2, z2) = node2
        return abs(x2-x)+abs(y2-y)+abs(z2-z)

    # Helper function to reconstruct the path
    def reconstructPath(self, cameFrom, currentNode):
        # pprint.pprint( cameFrom)

        paths = []
        allPaths = self.returnMultiplePaths(cameFrom, [currentNode], paths)
        return allPaths
        # path = [currentNode]
        # while currentNode in cameFrom.keys():
        #   currentNode = cameFrom[currentNode]
        #   path.append(currentNode)
        # return list(reversed(path))

    def returnMultiplePaths(self, cameFrom, path, paths):
        lastNode = path[-1]
        if len(paths) == 100:
            return paths
        if lastNode in cameFrom.keys():
            for nextNode in cameFrom[lastNode]:
                new_path = path + [nextNode]
                paths = self.returnMultiplePaths(cameFrom, new_path, paths)
        else:
            path = list(reversed(path))
            paths += [path]
        return paths


class Dijkstra(object):

    """Dijkstras algorithm

    Args:
        board (:obj: Board): the board on which to plan a path
        net (:obj: Net): a net object for which to plan a path
    """

    def __init__(self, board, net):
        super(Dijkstra, self).__init__()
        self.board = board
        self.net = net
        self.remaining = {}
        self.explored = {}

    """
    Create a path, returns a list of coordinates (including start and end gate coordinates), or empty list if no path exists.
    
    Returns: 
        list: path as an ordered list of coordinates, empty if no path exists.
    """

    def createPath(self):

        # define start/end gate
        start = self.board.gates[self.net.start_gate].getCoordinates()
        end = self.board.gates[self.net.end_gate].getCoordinates()

        self.remaining[start] = 0
        ended = False
        while not ended:
            newRemaining = {}
            for coord, val in self.remaining.iteritems():
                if end in self.board.getAllNeighbours(coord[0], coord[1], coord[2]):
                    ended = True
                rem = self.explore(coord, val)
                if rem == False:
                    path = []
                    return path
                newRemaining.update(rem)
                self.explored[coord] = val
            if newRemaining == {}:
                path = []
                return path
            self.remaining = newRemaining.copy()

        coord = end
        path = [coord]
        self.explored[end] = val + 1
        found = False
        
        while not start in self.board.getAllNeighbours(coord[0], coord[1], coord[2]):
            nextCoord = self.getLowestValue(coord)
            if not nextCoord:
                self.net.path = False
                return self.net
            path.append(nextCoord)
            coord = nextCoord
        path.append(start)

        return path

    """
    Explore the neighbours of coord, and set their distance. Returns the newly explored neighbours if there are any, 
    
    Args:
        coord (tuple): The coordinates (x,y,z) of a node to be explored
        val (int): The distance from start to coord
    
    Returns: 
        dict<tuple, int>: A dictionary with open neighbours of coord and the distance to them from start
    """

    def explore(self, coord, val):

        newRemaining = {}

        neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
        if len(neighbours) == 0:
            return False

        for neighbour in neighbours:
            if neighbour not in self.explored:
                # increase neighbour values based on current pos
                newRemaining[neighbour] = val + 1

        return newRemaining

    """
    Helper function to return the coordinates of the neighbour with the lowest distance to start
    
    Args:
        coord (tuple): The coordinate (x,y,z) around which to search for the lowest distance
    
    Returns: 
        tuple: the coordinate (x,y,z) with the lowest distance to start
    """

    def getLowestValue(self, coord):

        neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
        if len(neighbours) == 0:
            return False

        # set max value
        lowestValue = sys.maxint

        for neighbour in neighbours:
            if neighbour in self.explored:
                # find neighbour value
                value = self.explored[neighbour]
                # if current neighbour value is lowest till now
                if value < lowestValue:
                    lowestValue = value
                    bestCoord = neighbour

        return bestCoord


class GeneticOpt(object):

    """A genetic algorithm for optimizing a board and net configuration.

    Args:
        base_board (:obj: Board): The unoptimized board
        max_generations (int): The number of generations to simulate
        max_population (int): The size of the population
    """

    def __init__(self, alg_str, base_board, max_generations, max_population):
        super(GeneticOpt, self).__init__()
        self.alg_str = alg_str
        self.base_board = base_board
        self.max_generations = max_generations
        self.max_population = max_population
        self.base_score = self.base_board.getScore()[1]
        self.population = self.initPop()
        ts = time.time()
        self.resultFile = ('resources/' 
        					+ datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        					+ 'g' + str(self.max_generations)
        					+ 'p' + str(self.max_population) + '.csv')
        with open(self.resultFile, "a") as file:
        	file.write("iteration, max_score, min_score \n")

    def initPop(self):
        """Initialize the population

        Return:
            list: A list of <max_generation> deep-copied board objects
        """
        pop = []
        for i in range(self.max_population):
            pop.append((copy.deepcopy(self.base_board), self.base_score))
        return pop

    def run(self):
        """Run the algorithm

        Return:
            :obj:Board: The board with the lowest score from the last generation
        """
        for i in range(self.max_generations):
            print "In Generation", i
            newPop = self.iteration(self.population)
            sortedPop = self.sortPop(newPop)
            print self.population[0][1]
            self.writeIterationResults(i)
            # print [str(net.start_gate)+">"+str(net.end_gate)+":"+str(len(net.path)) for i, net in self.population[0][0].nets.iteritems()]
            # print [score for l,score in self.population]
            killedPop = self.killPop(sortedPop)
            self.population = self.repopulate(killedPop)
        print "Improved from", self.base_score, 'to', self.population[0][1]
        return self.population[0][0]

    def iteration(self, population):
        """Runs the A* path finding algorithm for each member of the population

        Args:
            population (int): The number of individuals in each generation

        Return:
            list :A list with the new population as tuples of (board, score)
        """
        newPop = []
        for i, (board, score) in enumerate(population):
            if i % (population/10) == 0:
                print '.',
                sys.stdout.flush()
            net = random.choice(board.nets)
            oldPath = copy.deepcopy(net.path)
            # print net.start_gate, net.end_gate
            # print [len(net.path) for i,net in board.nets.iteritems()]
            board.removeNetPath(net)
            if self.alg_str == 'astar':
                astar = AStarAllPaths(board, net)
                paths = astar.createPath()
                path = random.choice(paths)

                lengths = [len(p) for p in paths]
                # print max(lengths), min(lengths), len(paths)
            else:
                bfs = BreadthFirst(net, board)
                paths = []
                paths = bfs.createPaths()
                # for p in paths:
                #   print p[0]
                if len(paths) > 0:
                    path = random.choice(paths)
                else:
                    print "No paths found"
                    path = []

            net.path = path
            if len(net.path) == 0:
                # print '\nFailed planning a better path for net', i, '!'
                net.path = oldPath
                if not board.setNetPath(net):
                    print "OLD SHIT IS WRONG!!"
                newScore = board.getScore()[1]
                newPop.append((board, newScore))
            else:
                if not board.setNetPath(net):
                    print i, "SHIT IS WRONG!!!"
                newScore = board.getScore()[1]
                newPop.append((board, newScore))
        return newPop

    def sortPop(self, pop):
        return sorted(pop, key=lambda tup: tup[1])

    def killPop(self, pop):
        return pop[:len(pop)/2]

    def repopulate(self, pop):
        return copy.deepcopy(pop) + copy.deepcopy(pop)

    def writeIterationResults(self, iteration):
    	with open(self.resultFile, "a") as file:
    		max_score = self.population[0][1]
    		min_score = self.population[len(self.population)-1][1]
    		line = str(iteration) + ',' + str(max_score) + ',' + str(min_score) +'\n'
    		file.write(line)


if __name__ == '__main__':
    from helpers import *

    netlist = [(15, 8), (3, 15), (15, 5), (20, 19), (23, 4), (5, 7), (1, 0), (15, 21), (3, 5), (7, 13), (3, 23), (23, 8), (22, 13), (15, 17),
               (20, 10), (13, 18), (19, 2), (22, 11), (10, 4), (11, 24), (2, 20), (3, 4), (16, 9), (19, 5), (3, 0), (6, 14), (7, 9), (9, 13), (22, 16), (10, 7)]
    board = runAlgorithm('astar', 0, netlist, 5, recursive=True)
    print '\nold board score:', board.getScore()
    # net = board.nets[0]
    # print 'Planning new path from',net.start_gate, 'to', net.end_gate
    # print 'old path', net.path
    # board.removeNetPath(net)
    # bfa = BreadthFirst(board.getElementAt(net.start_gate[0],net.start_gate[1],net.start_gate[2]).gate_id, board.getElementAt(net.end_gate[0],net.end_gate[1],net.end_gate[2]).gate_id, board)
    # paths = bfa.solve()
    # print 'found', len(paths), 'possible paths'
    # print 'The first is:', paths[0]
    hc = HillClimber(board)
    newBoard = hc.solve()
    print 'new board score:', newBoard.getScore()
    # v = Visualizer(newBoard)
    # v.start()
