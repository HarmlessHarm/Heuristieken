import pprint

# class Board(object):
# 	"""docstring for Board"""
# 	def __init__(self, x_dim, y_dim):
# 		# # super(Board, self).__init__()
# 		# self.dimensions = (x_dim, y_dim)
# 		self.board = [[[None for k in range(1)] for j in range(y_dim)] for i in range(x_dim)]
# =======
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

	def printBoard(self):
		pprint.pprint(self.board)


if __name__ == '__main__':
	b = Board(3,3)

	pprint.pprint(b.board)