import pprint, numpy as np

class Board(object):
	"""docstring for Board"""
	def __init__(self, x_dim, y_dim, z_dim=1):
		super(Board, self).__init__()
		self.x_dim = x_dim
		self.y_dim = y_dim
		self.z_dim = z_dim

		self.board = np.zeros((y_dim,x_dim,z_dim))

	def getElementAt(self, x, y, z):
		return self.board[x][y][z]

	def setElementAt(self, x, y, z, gateID):
		if self.getElementAt(x, y, z) == 0:
			self.board[x][y][z] = gateID

	def prettyPrintLayer(self, layer):
		print b.board[:,:,layer]

	def printBoard(self):
		for z in range(self.z_dim):
			self.prettyPrintLayer(z)
			print "\n"


if __name__ == '__main__':
	b = Board(3,4,3)
	b.printBoard()