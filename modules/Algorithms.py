# -*- coding: utf-8 -*-
import numpy as np
from random import shuffle
import sys
from Objects import *
from Visualizer import *
from Sorter import *
import copy
import pprint


class EasyPath(object):

    """Attempt at creating a path, doesn't work!"""

    def __init__(self, board, net):
        super(EasyPath, self).__init__()
        self.board = board
        self.net = net

    def createPath(self):
        board = self.board
        start = board.gates[self.net.start_gate].getCoordinates()
        end = board.gates[self.net.end_gate].getCoordinates()

        curPos = start
        path = [start]

        if curPos[2] < self.net.net_id:
            PHASE = 'UP'
        else:
            PHASE = 'LAT'

        while not self.checkAdjacent(curPos, end):
            # if checkAdjacent(curPos, end):
            #   return net

            dX = end[0] - curPos[0]
            dY = end[1] - curPos[1]

            # print dX, dY

            # print curPos[2], PHASE, net.net_id
            # UP PHASE
            if PHASE == 'UP':
                nextPos = self.goUp(curPos, board)
                if not nextPos:
                    nextPos = self.goX(curPos, dX, board)
                    if not nextPos:
                        nextPos = self.goY(curPos, dY, board)
                        if not nextPos:
                            nextPos = self.goNotX(curPos, dX, board)
                            if not nextPos:
                                nextPos = self.goNotY(curPos, dY, board)
                if nextPos == False:
                    path = []
                    return path
                path.append(nextPos)
                board.setElementAt(
                    self.net, nextPos[0], nextPos[1], nextPos[2])
                curPos = nextPos
                if curPos[2] == self.net.net_id:
                    PHASE = 'LAT'
            # LATERAL PHASE
            if PHASE == 'LAT':
                nextPos = self.goX(curPos, dX, board)
                if not nextPos:
                    nextPos = self.goY(curPos, dY, board)
                    if not nextPos:
                        nextPos = self.goNotX(curPos, dX, board)
                        if not nextPos:
                            nextPos = self.goNotY(curPos, dY, board)

                if nextPos == False:
                    path = []
                    return path
                path.append(nextPos)
                board.setElementAt(
                    self.net, nextPos[0], nextPos[1], nextPos[2])
                curPos = nextPos
                if dX == 0 and dY == 0:
                    PHASE = 'DOWN'
            # DOWN PHASE
            if PHASE == 'DOWN':
                nextPos = False
                if dX != 0:
                    nextPos = self.goX(curPos, dX, board)
                elif dY != 0:
                    nextPos = self.goY(curPos, dY, board)
                if not nextPos:
                    nextPos = self.goDown(curPos, board)
                    if not nextPos:
                        nextPos = self.goX(curPos, dX, board)
                        if not nextPos:
                            nextPos = self.goY(curPos, dY, board)
                            if not nextPos:
                                nextPos = self.goNotX(curPos, dX, board)
                                if not nextPos:
                                    nextPos = self.goNotY(curPos, dY, board)
                if nextPos == False:
                    path = []
                    return path
                path.append(nextPos)
                board.setElementAt(
                    self.net, nextPos[0], nextPos[1], nextPos[2])
                curPos = nextPos
                if curPos[2] == 0:
                    PHASE = 'LAT'
        path.append(end)
        return path

    def checkAdjacent(self, curPos, end):
        # Same plane

        if curPos[2] == end[2]:

            if curPos[1] == end[1]:
                if abs(end[0] - curPos[0]) == 1:
                    return True
            elif curPos[0] == end[0]:
                if abs(end[1] - curPos[1]) == 1:
                    return True
        elif curPos[0] == end[0] and curPos[1] == end[1] and abs(end[2] - curPos[2]) == 1:
            return True
        else:
            return False

    def goUp(self, curPos, board):
        newPos = curPos[0], curPos[1], curPos[2] + 1
        if board.isEmpty(newPos):
            return newPos
        else:
            return False

    def goDown(self, curPos, board):
        newPos = curPos[0], curPos[1], curPos[2] - 1
        if board.isEmpty(newPos):
            return newPos
        else:
            return False

    def goX(self, curPos, dX, board):
        if dX != 0:
            newPos = curPos[0] + np.sign(dX), curPos[1], curPos[2]
        else:
            newPos = curPos[0] + 1, curPos[1], curPos[2]
        if board.isEmpty(newPos):
            return newPos
        else:
            return False

    def goY(self, curPos, dY, board):
        if dY != 0:
            newPos = curPos[0], curPos[1] + np.sign(dY), curPos[2]
        else:
            newPos = curPos[0], curPos[1] + 1, curPos[2]
        if board.isEmpty(newPos):
            return newPos
        else:
            return False

    def goNotX(self, curPos, dX, board):
        if dX != 0:
            newPos = curPos[0] - np.sign(dX), curPos[1], curPos[2]
        else:
            newPos = curPos[0] - 1, curPos[1], curPos[2]
        if board.isEmpty(newPos):
            return newPos
        else:
            return False

    def goNotY(self, curPos, dY, board):
        if dY != 0:
            newPos = curPos[0], curPos[1] - np.sign(dY), curPos[2]
        else:
            newPos = curPos[0], curPos[1] - 1, curPos[2]
        if board.isEmpty(newPos):
            return newPos
        else:
            return False

    def goBack(self, curPos, board, net):
        # Does not work as expected
        if len(net.path) > 1:
            print '\ngoBack'
            print net.path[-1], len(net.path)
            net.path.remove(net.path[-1])
            print net.path[-1], len(net.path)
            board.removeElementAt(curPos)
            board.setElementAt(-1, curPos[0], curPos[1], curPos[2])
            prevPos = net.path[-1]
            print prevPos
            return prevPos
        return False


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


class DepthFirst(object):

    """Depthfirst search through netlist orderings in an attempt to find a solvable ordering.

    Args:
        board (:obj: Board): A board object on which to place the nets.
        netlist (list<tuples>): A list of tuples with start and end gates [(start1, end1), (start2, end2), ..., (startN, endN)]
    """

    def __init__(self, board, netlist):
        super(DepthFirst, self).__init__()
        self.netlist = netlist
        self.board = board

    """Iterates depth-first through netlist orderings, aided by sorting the nets in netlist (the heuristic).
    
    Returns:
        Board: A board with paths for all nets in netlist.
    """

    def solve(self):
        tree = []
        discovered = []

        # create TreeNode object for root
        start_node = TreeNode(self.board, 'start', self.netlist)
        tree.append(start_node)
        iterations = 0

        while len(tree) != 0:
            currentNode = tree.pop()
            iterations += 1
            print 'size of stack:', len(tree), ',nets to be solved:', len(currentNode.netlist)

            if len(currentNode.netlist) == 0:
                n = self.reconstructNetlist(currentNode)
                print 'Finding the right ordering took', iterations, 'iterations'
                return currentNode.board, n

            if currentNode not in discovered:
                discovered.append(currentNode)

                # sort netlist by distance (low - high)
                s = Sorter(currentNode.netlist, currentNode.board)
                currentNode.netlist = s.sortNetlistByDistance()

                # check every net in netlist
                for (start, end) in currentNode.netlist:
                    new_board = copy.deepcopy(currentNode.board)
                    new_netlist = copy.deepcopy(currentNode.netlist)
                    new_netlist.remove((start, end))

                    # create net object for current net
                    net = Net(start, end, len(currentNode.netlist))

                    # run the A* algorithm
                    alg = AStar(new_board, net)
                    path = alg.createPath()

                    # if no path can be found, proceed to next net in netlist.
                    if path == []:
                        continue

                    # add the net and path to the board
                    net.path = path
                    new_board.nets[net.net_id] = net
                    new_board.setNetPath(net)

                    # create new TreeNode object for current net
                    new_node = TreeNode(new_board, currentNode, new_netlist)
                    tree.append(new_node)

        # no solution found, practically unreachable due to the size of the
        # tree
        return False

    def reconstructNetlist(self, currentNode):
        """Reconstructs the netlist from a TreeNode

        Args:
            currentNode (:obj:Treenode): A node from which to traverse back to start.

        Returns:
            list<Tuple>: An ordered netlist
        """
        netlist = []
        lastNode = currentNode.previousNode
        while lastNode != 'start':
            # Het laatst opgeloste net is het verschil tussen de overgebleven
            # netlist van currentnode en lastnode
            net = set(lastNode.netlist).symmetric_difference(
                currentNode.netlist).pop()
            netlist.append(net)
            currentNode = lastNode
            lastNode = currentNode.previousNode
        reversed(netlist)
        return netlist


class BreadthFirst(object):

    """
    BreadthFirst algorithm for finding a set of paths
    :param net: A Net object for which to find a path
    :param board: A Board object on which to plan the path
    """

    def __init__(self, net, board):
        super(BreadthFirst, self).__init__()
        self.start = board.gates[net.start_gate]
        self.end = board.gates[net.end_gate]
        self.board = copy.deepcopy(board)
        (sx, sy, sz) = self.start.getCoordinates()
        (ex, ey, ez) = self.end.getCoordinates()
        self.max = abs(ex-sx)+abs(ey-sy)+abs(ez-sz)+2

    """
    Creates at least all shortest paths, and maybe more. It runs for 10 percent more iterations than needed
    so that it finds some slightly longer paths as well.
    
    :return: A list of ordered lists of coordinates, paths between start and end of net. 
    """

    def createPaths(self):
        queue = []
        visited = []
        dictPreviousNode = {}
        counter = 0
        maximum = -1
        # allPaths = []

        queue.insert(0, self.start.getCoordinates())
        while len(queue) != 0:
            currentNode = queue.pop()
            if currentNode not in visited:
                # vind alle buren
                (x, y, z) = currentNode
                (sx, sy, sz) = self.start.getCoordinates()
                currentDepth = abs(x-sx)+abs(y-sy)+abs(z-sz)

                # If currentNode is adjacent to the end node, we are almost
                # done!
                if self.end.getCoordinates() in self.board.getAllNeighbours(x, y, z):
                    if self.end.getCoordinates() not in dictPreviousNode.keys():
                        dictPreviousNode[
                            self.end.getCoordinates()] = [currentNode]
                    else:
                        dictPreviousNode[
                            self.end.getCoordinates()].append(currentNode)
                    # return self.reconstructPaths(dictPreviousNode,
                    # [self.end.getCoordinates()])

                if currentDepth > self.max:
                    allPaths = self.reconstructPaths(
                        dictPreviousNode, [self.end.getCoordinates()], [])
                    return allPaths

                # voeg buren toe aan queue als je deze nog niet gecheckt hebt
                neighbours = self.board.getOpenNeighbours(x, y, z)
                for neighbour in neighbours:
                    if neighbour in visited:
                        continue

                    if neighbour not in dictPreviousNode.keys():
                        dictPreviousNode[neighbour] = [currentNode]
                    else:
                        dictPreviousNode[neighbour].append(currentNode)

                    queue.insert(0, neighbour)
                visited.append(currentNode)
                counter += 1
        allPaths = self.reconstructPaths(
            dictPreviousNode, [self.end.getCoordinates()], [])
        return allPaths

    def reconstructPaths(self, cameFrom, path, paths=[]):
        """Reconstructs all paths between goal and start node

        Args:
            cameFrom (dict): a dictionary that maps each coordinate to a list of coordinates from which this can be reached
            path (list): a path
            paths (list): a list of paths

        Return:
            list: a list of all paths.      
        """
        lastNode = path[-1]
        if lastNode in cameFrom.keys():
            for nextNode in cameFrom[lastNode]:
                new_path = path + [nextNode]
                paths = self.reconstructPaths(cameFrom, new_path, paths)
        else:
            path = list(reversed(path))
            paths += [path]
        return paths


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
            if i % 10 == 0:
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


class HillClimber(object):

    """Hillclimber Algorithm NOT FINISHED
    """

    def __init__(self, board, iterations=1):
        super(HillClimber, self).__init__()
        self.board = board
        self.iterations = iterations

    def solve(self):

        for i in range(self.iterations):

            print 'hillclimbing iteration:', i
            random_net = random.choice(self.board.nets)
            print 'changing net:', random_net.net_id
            print 'With current path:', random_net.path
            oldpath = copy.deepcopy(random_net.path)

            startGateID = self.board.getElementAt(
                random_net.start_gate[0], random_net.start_gate[1], random_net.start_gate[2]).gate_id
            endGateID = self.board.getElementAt(
                random_net.end_gate[0], random_net.end_gate[1], random_net.end_gate[2]).gate_id

            if not self.board.removeNetPath(random_net):
                print 'Failed removing old path!'

            breadthFirstAlgorithm = BreadthFirst(
                startGateID, endGateID, self.board)
            possibleNewPaths = breadthFirstAlgorithm.solve()

            print 'found', len(possibleNewPaths), 'paths'
            if len(possibleNewPaths) < 1:
                continue

            random_net.path = random.choice(possibleNewPaths)
            print 'New path is:', random_net.path

            # astar = AStar(self.board, random_net)
            # print random_net.path
            # random_net = astar.createPath(random_net.start_gate, random_net.end_gate, 'no_bias')
            # print random_net.path

            if len(random_net.path) <= len(oldpath):
                print 'selected path is smaller than or equal to original path'
                if not self.board.setNetPath(random_net):
                    print 'Failed placing path!'
                    break
            else:
                random_net.path = oldpath
                if not self.board.setNetPath(random_net):
                    print 'Failed restoring old path!'
                    break

        return self.board

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
