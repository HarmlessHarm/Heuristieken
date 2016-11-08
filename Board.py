class Board(object):
	"""docstring for Board"""
	def __init__(self, x_dim, y_dim):
		super(Board, self).__init__()
		z_dim = 1
		self.dimensions = (x_dim, y_dim, z_dim)
		self.board = [[[0]*z_dim]*y_dim]*x_dim

	def getElementAt(self, x, y, z):
		return self.board[x][y][z]

	def setElementAt(self, x, y, z, gateID):
		if self.getElementAt(x, y, z) == 0:
			self.board[x][y][z] = gateID

	def prettyPrintLayer(self, layer):
		for y in range(0, self.dimensions[1]):
			print(self.board[:][y])