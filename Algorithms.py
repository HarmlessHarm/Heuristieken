


class EasyPath():
	"""docstring for EasyPath"""
	def __init__(self, board):
		super(EasyPath, self).__init__()
		self.board = board
		
	def getPath(self, start, end):
		path = []
		print start, end
		dX = end[0] - start[0]
		dY = end[1] - start[1]

