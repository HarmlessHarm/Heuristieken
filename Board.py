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

    def setElementAt(self, x, y, z, gateID):
        if self.getElementAt(x, y, z) == 0:
            self.board[x][y][z] = gateID

    def getLayer(self, layer):
        return self.board[:, :, layer]

    def printBoard(self):
        for z in range(self.z_dim):
            print self.getLayer(z)
            print "\n"


if __name__ == '__main__':
    b = Board(3, 5, 3)
    b.printBoard()
