import pprint
import numpy as np


class Board(object):

    """docstring for Board"""

    def __init__(self, x_dim, y_dim, z_dim=1):
        super(Board, self).__init__()
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim

        self.board = np.zeros((x_dim, y_dim, z_dim))

    def getElementAt(self, x, y, z):
        return self.board[x, y, z]

    def setElementAt(self, obj,  x, y, z=0):
        if self.getElementAt(x, y, z) == 0:
            self.board[x][y][z] = obj
        else:
            return False

    def getLayer(self, layer):
        return self.board[:, :, layer]

    def getDimensions(self):
    	return (self.x_dim, self.y_dim, self.z_dim)
    	
    def printBoard(self):
        for z in range(self.z_dim):
            print self.getLayer(z)
            print "\n"


class Gate(object):

    """ Docstring for Gate """

    def __init__(self, x, y, gate_id):
        self.x = x
        self.y = y
        self.gate_id = gate_id

    def getCoordinates(self, coordinates):
        return (x, y)


class Net:

# 	""" Docstring for Net """

	def __init__(self, identifier, start, end):
		self.identifier = identifier
		self.start = start
		self.end = end
		self.path = []


if __name__ == '__main__':
    b = Board(3, 5, 3)
    b.printBoard()
