import sys

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
		ended = False
		while not ended:
			newRemaining = {}
			for coord, val in self.remaining.iteritems():
				if end in self.board.getAllNeighbours(coord[0],coord[1],coord[2]):
					ended = True
				newRemaining.update(self.explore(coord, val))
				self.explored[coord] = val

			self.remaining = {}
			self.remaining = newRemaining.copy()


		coord = end
		self.net.addPos(end)
		self.explored[end] = val + 1
		while coord != start:

			nextCoord = self.getLowestValue(coord)
			self.net.addPos(nextCoord)
			print self.net.path
			coord = nextCoord

		return self.net


	def explore(self, coord, val):

		newRemaining = {}

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])

		for neighbour in neighbours:
			if neighbour not in self.explored:
				newRemaining[neighbour] = val + 1

		return newRemaining


	def getLowestValue(self, coord):

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
		print neighbours
		lowestValue = sys.maxint
		for neighbour in neighbours:
			if neighbour in self.explored:
				value = self.explored[neighbour]
				if value < lowestValue:
					lowestValue = value
					bestCoord = neighbour

		return bestCoord
