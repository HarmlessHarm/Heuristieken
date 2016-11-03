

class Board(object):
	"""docstring for Board"""
	def __init__(self, x_dim, y_dim):
		super(Board, self).__init__()
		self.dimensions = (x_dim, y_dim)
		self.board = [][][]

	def getElementAt(self, x, y, z):
		return self.board[x][y][z]

	def setElementAt(self, x, y, z, object):
		if getElementAt(x, y, z) == None:
