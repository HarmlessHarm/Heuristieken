# -*- coding: utf-8 -*-
from __future__ import division
import pprint
import copy
import numpy as np

class Board(object):

    """
    Board object in which all data is stored
    """

    def __init__(self, x_dim, y_dim, z_dim=10):
        """
        Creates a Board object with certain dimensions and initializes a 3d 
        numpy array which represents the circuit.
        Also contains a dictionary for where all gates and nets are stored with
        their id as key

        Args:
            x_dim (int): Width of the circuit
            y_dim (int): Height of the circuit
            z_dim (int, optional): Maximum depth of the circuit. Defaults to 10
        """
        super(Board, self).__init__()
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim
        self.board = np.zeros((x_dim, y_dim, z_dim), dtype=object)
        self.gates = {}
        self.nets = {}

    def getElementAt(self, x, y, z):
        """
        Returns the object that is positioned at a certain position on the board.

        Args:
            x,y,z (int): x,y,z coordinate.
        Returns:
            Object at certain coordinate.

            Returns False if one of the provided coordinates are out of bound 
            of the board.
        """
        if 0 <= x < self.x_dim and 0 <= y < self.y_dim and 0 <= z < self.z_dim:
            return self.board[x, y, z]
        return False

    def isEmpty(self, xyz):
        """ 
        Checks if board at a certain position is empty.

        Args:
            xyz (int tuple): Contains the x, y and z coordinates.
        Returns:
            bool: True if cell is empty, False otherwise.
        """
        x, y, z = xyz
        if self.getElementAt(x, y, z) is 0:
            return True
        else:
            return False

    def setElementAt(self, obj,  x, y, z=0):
        """
        Puts an object in a cell at a certain coordinate if the cell is empty.
    
        This function can be used to set Gate and Net objects.

        Args:
            obj (object): Object that is set at position.
            x,y,z (int): x,y,z coordinate. Z Defaults at 0
        Returns:
            bool: True if element is set. False if target 
        """
        if self.isEmpty((x, y, z)):
            self.board[x, y, z] = obj
            return True
        else:
            return False

    def removeElementAt(self, xyz):
        """
        Deletes object from cell at certain coordinate.

        Args:
            xyz (int tuple): Contains the x, y and z coordinates.
        Returns:
            bool: True when finished.    
        """
        self.board[xyz] = 0
        return True

    def getDimensions(self):
        """
        Returns x, y and z dimensions of the board.
        """
        return (self.x_dim, self.y_dim, self.z_dim)

    def getAllNeighbours(self, x, y, z):
        """
        Returns a list with all neighbours of a cell at certain coordinate.

        Uses a difference vector to get neighbours in x,y,z directions.
        Checks if neighbour is in bound of the board.

        Args:
            x,y,z (int): x,y,z coordinates
        Returns: 
            neighbours (tuple list): List of all coordinates of the neighbours 
                of the cell.
        """
        xyz = (x, y, z)
        neighbours = []
        xs = [-1, 1, 0, 0, 0, 0]
        ys = [0, 0, -1, 1, 0, 0]
        zs = [0, 0, 0, 0, -1, 1]

        for dif_tuple in zip(xs, ys, zs):
            (nx, ny, nz) = [sum(x) for x in zip(xyz, dif_tuple)]
            if 0 <= nx < self.x_dim and 0 <= ny < self.y_dim and \
                0 <= nz < self.z_dim and xyz != (nx, ny, nz):
                neighbours.append((nx, ny, nz))
        return neighbours

    def getOpenNeighbours(self, x, y, z):
        """
        Returns a list of all empty neighbours of a cell at a certain coordinate.

        Args:
            x,y,z (int): x,y,z coordinates
        Returns:
            openNeighbours (tuple list): List of all coordinates of the empty 
                neighbours of the cell.
        """
        neighbours = self.getAllNeighbours(x, y, z)
        openNeighbours = []
        for n in neighbours:
            if self.isEmpty(n):
                openNeighbours.append(n)
        return openNeighbours

    def getScore(self):
        """
        Returns the number of paths and the score of the board.

        Calculates the score by summing the length of each path of a net 
        on the board.
        The number of paths represents the amount of completed nets. Optimally
        this should be the same as the length of the netlist.

        Returns:
            pathCount (int): Number of paths
            score (int): Score of a board
        """
        score = 0
        pathCount = 0
        for i in self.nets:
            path = self.nets[i].path
            if type(path) is list and path != []:
                pathCount += 1
                score += len(path)-1
        return pathCount, score

    def getMinimumScore(self):
        """
        Returns the minimum score a board can have.

        Calculates the score by using the manhattan distance between the start
        and end points of each net in the board. This score is used as a 
        comparison for the real score.

        Returns:
            score (int): 
        """
        score = 0
        for netID, netObject in self.nets.iteritems():
            # returns all net objects
            startId = netObject.start_gate
            endId = netObject.end_gate

            startGate = self.gates[startId]
            endGate = self.gates[endId]

            # get x, y, z coordinates both gates
            startCor = startGate.getCoordinates()
            endCor = endGate.getCoordinates()

            score += abs(startCor[0] - endCor[0]) + abs(endCor[1] - endCor[1])

        return score

    def getRelativeScore(self):
        """
        Returns the relative score between minimum and actual score.
        """
        minimum = self.getMinimumScore()
        current = self.getScore()
        # check how many times current score fits into min. score
        return current[1] / minimum

    def setNetPath(self, net):
        """
        Iterates through the path in a net and sets the corresponding cells.

        Sets Net objects at the corresponding cells.

        Args:
            net (Net): net that is iterated.
        Returns:
            bool: True if completed. False if a position isn't valid.
        """
        toremove = net.path[1:-1]
        for i, (x, y, z) in enumerate(toremove):
            if not self.setElementAt(net, x, y, z):
                print 'False @:',(x,y,z)
                if i != 0:
                    for coord in toremove[:i]:
                        self.removeElementAt(coord)
                return False
        return True

    def removeNetPath(self, net):
        """
        Removes all objects from the board at the corresponding path.

        Iterates over the path and removes objects from the board. 
        Empties the path attribute of the Net opbject.

        Args:
            net (Net): net that is used.
        Returns:
            bool: True if completed. False if not removed.
        """
        if net.path != []:
            for (x, y, z) in net.path[1:-1]:
                self.removeElementAt((x, y, z))
            net.path = []
            return True
        else:
            return False

class Gate(object):

    """ 
    Gate object with coordinates and identifier

    Attributes:
        gate_id (int): Unique identifier of the gate.
        x, y, z (int): Coordinates of the gate.
    """

    def __init__(self, gate_id, x, y, z=0):
        super(Gate, self).__init__()
        self.gate_id = gate_id
        self.x = x
        self.y = y
        self.z = z

    def getCoordinates(self):
        """
        Returns coordinates of gate
        """
        return (self.x, self.y, self.z)

class Net(object):

    """
    Net object with identifier, start and end gate identifiers ano ffd the path.

    Attributes:
        start_gate, end_gate (int): Gate identifiers of the start and end gates.
        net_id (int): Unique Net identifier.
        path (tuple list): List of coordinates that make up the path of a net.
    """

    def __init__(self, start_gate, end_gate, net_id):
        super(Net, self).__init__()
        self.net_id = net_id
        self.start_gate = start_gate
        self.end_gate = end_gate
        self.path = []

    def addPos(self, xyz):
        """
        Adds a new coordinate to the path attribute
        Args:
            xyz (int tuple): New coordinate to add to path
        """
        x, y, z = xyz
        self.path.append((x, y, z))


class TreeNode(object):

    """
    Node in search tree

    Attributes:
        board (Board): Board object in node
        previousNode (TreeNode): Parent of the TreeNode object
        netlist (tuple list): Netlist in node
    """

    def __init__(self, board, previousNode, netlist):
        super(TreeNode, self).__init__()
        self.board = board
        self.previousNode = previousNode
        self.netlist = netlist
