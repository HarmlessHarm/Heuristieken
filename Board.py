import pprint

class Board(object):
	"""docstring for Board"""
	def __init__(self, x_dim, y_dim):
		# # super(Board, self).__init__()
		# self.dimensions = (x_dim, y_dim)
		self.board = [[[None for k in range(1)] for j in range(y_dim)] for i in range(x_dim)]

	def getElementAt(self, x, y, z):
		return self.board[x][y][z]

	def setElementAt(self, x, y, z, object):
		pass

	def printBoard(self):
		pprint.pprint(self.board)


if __name__ == '__main__':
	b = Board(3,3)

	pprint.pprint(b.board)