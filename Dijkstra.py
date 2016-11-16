class Dijkstra(object):
	"""docstring for Dijkstra"""
	def __init__(self, board, net):
		super(Dijkstra, self).__init__()
		self.board = board
		self.net = net
		self.remaining = {}
		self.explored = {}

	def createPath(self):

		start = self.board.gates[self.net.start_gate]
		end = self.board.gates[self.net.end_gate]

		self.remaining[start] = 0

		while not self.board.gates[self.net.end_gate] in self.explored.keys():
			
			newRemaining = {}
			for coord, val in self.remaining:
				newRemaining.update(self.explore(coord, val))
				self.explored[coord] = val

			self.remaining = {}
			self.remaining = newRemaining.copy()

		coord = end
		while coord != start:

			nextCoord = self.getLowestValue(coord)
			self.net.addPos(nextCoord)
			coord = nextCoord

		return self.net


	def explore(self, coord, val):

		newRemaining = {}

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])

		for neighbour in neighbours:
			newRemaining[neighbour] = val + 1

		return newRemaining


	def getLowestValue(self, coord):

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])

		lowestValue = sys.maxint
		for neighbour in neighbours:

			value = self.explored[neighbour]
			if value < lowestValue:
				lowestValue = value
				bestCoord = neighbour

		return bestCoord
