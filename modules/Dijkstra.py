from Objects import *
import numpy as np
import sys
import copy

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
                x, y, z = coord
                if end in self.board.getAllNeighbours(x, y, z):
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

        (x, y, z) = end
        path = [coord]
        self.explored[end] = val + 1
        found = False
        # x,y,z = coord
        while not start in self.board.getAllNeighbours(x, y, z):
            nextCoord = self.getLowestValue((x, y, z))
            if not nextCoord:
                self.net.path = False
                return self.net
            path.append(nextCoord)
            (x, y, z) = nextCoord

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

