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
				rem = self.explore(coord, val)
				if rem == False:
					self.net.path = False
					return self.net
				newRemaining.update(rem)
				self.explored[coord] = val
			if newRemaining == {}:
				self.net.path = False
				return self.net
			self.remaining = newRemaining.copy()

		coord = end
		self.net.addPos(end)
		self.explored[end] = val + 1
		found = False
		while not found:
			if start in self.board.getAllNeighbours(coord[0],coord[1],coord[2]):
				found = True
				nextCoord = start
			else:
				nextCoord = self.getLowestValue(coord)
				if not nextCoord:
					self.net.path = False
					return self.net

			self.net.addPos(nextCoord)	
			self.board.setElementAt(self.net, coord[0], coord[1], coord[2])
			coord = nextCoord

		return self.net


	def explore(self, coord, val):

		newRemaining = {}

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
		if len(neighbours) == 0:
			return False

		for neighbour in neighbours:
			if neighbour not in self.explored:
				newRemaining[neighbour] = val + 1

		return newRemaining


	def getLowestValue(self, coord):

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
		if len(neighbours) == 0:
			return False
		lowestValue = sys.maxint
		for neighbour in neighbours:
			if neighbour in self.explored:
				value = self.explored[neighbour]
				if value < lowestValue:
					lowestValue = value
					bestCoord = neighbour

		return bestCoord
