import pprint
import copy
import numpy as np

class Board(object):
    """docstring for Board"""

    def __init__(self, x_dim, y_dim, z_dim=1):
        super(Board, self).__init__()
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim
        self.board = np.zeros((x_dim, y_dim, z_dim), dtype=object)
        self.gates = {}
        self.nets = {}

    def getElementAt(self, x, y, z):
        if  0 <= x < self.x_dim and 0 <= y < self.y_dim and 0 <= z < self.z_dim:
            return self.board[x, y, z]
        return False

    def isEmpty(self, xyz):
        x, y, z = xyz
        if self.getElementAt(x, y, z) is 0:
            return True
        else:
            return False

    def setElementAt(self, obj,  x, y, z=0):
        if self.isEmpty((x,y,z)):
            self.board[x, y, z] = obj
            return True
        else:
            return False

    def removeElementAt(self, xyz):
        self.board[xyz] = 0
        return True

    def addLayer(self):
        x, y = (self.x_dim, self.y_dim)
        layer = np.zeros((x,y,1))
        self.z_dim += 1
        self.board = np.append(self.board, layer, axis=2)

    def getLayer(self, layer):
        return self.board[:, :, layer]

    def getDimensions(self):
        return (self.x_dim, self.y_dim, self.z_dim)

    def getAllNeighbours(self, x, y, z):
        xyz = (x,y,z)
        neighbours = []
        xs = [-1,1,0,0,0,0]
        ys = [0,0,-1,1,0,0]
        zs = [0,0,0,0,-1,1]
        
        for dif_tuple in zip(xs,ys,zs):
            (nx,ny,nz) = [sum(x) for x in zip(xyz, dif_tuple)]
            if 0 <= nx < self.x_dim and 0 <= ny < self.y_dim and 0 <= nz < self.z_dim and xyz != (nx,ny,nz):
                neighbours.append((nx,ny,nz))
        return neighbours
        
    def getOpenNeighbours(self, x, y, z):
        neighbours = self.getAllNeighbours(x,y,z)
        openNeighbours = []
        for n in neighbours:
            if self.isEmpty(n):
                openNeighbours.append(n)
        return openNeighbours

    def getScore(self):
        score = 0
        pathCount = 0
        for i in self.nets:
            path = self.nets[i].path
            if type(path) is list and path != []:
                pathCount += 1
                score += len(path)-1
        return pathCount, score
        
    def printBoard(self):
        for z in range(self.z_dim):
            print 'layer',z
            print self.getLayer(z)
            print "\n"

    def setNetPath(self, net):
        for (x,y,z) in net.path[1:-1]:
            if not self.setElementAt(net, x,y,z):
                return False
        return True

    def removeNetPath(self, net):
        if net.path != []:
            for (x,y,z) in net.path[1:-1]:
                self.removeElementAt((x,y,z))
            net.path = []
            return True
        else: 
            return False


    def copy(self):
        newBoard = type(self)(self.x_dim, self.y_dim, self.z_dim)
        newArray = np.deepcopy(self.board)
        newGates = copy.deepcopy(self.gates)
        newNets = copy.deepcopy(self.nets)
        newBoard.board = newArray
        newBoard.gates = newGates
        newBoard.nets = newNets
        return newBoard


class Gate(object):

    """ Docstring for Gate """

    def __init__(self, gate_id, x, y, z=0):
        super(Gate, self).__init__()
        self.gate_id = gate_id
        self.x = x
        self.y = y
        self.z = z

    def getCoordinates(self, coordinates):
        return (x, y, z)

class Net(object):
    """ Docstring for Net """

    def __init__(self, start_gate, end_gate, net_id):
        super(Net, self).__init__()
        self.net_id = net_id
        self.start_gate = start_gate
        self.end_gate = end_gate
        self.path = []

    def addPos(self,xyz):
        x, y ,z = xyz
        self.path.append((x,y,z))

class TreeNode(object):
    """docstring for TreeNode"""
    def __init__(self, board, previousNode, netlist):
        super(TreeNode, self).__init__()
        self.board = board
        self.previousNode = previousNode
        self.netlist = netlist