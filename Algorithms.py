import numpy as np


class EasyPath(object):
	"""docstring for EasyPath"""
	def __init__(self, board):
		super(EasyPath, self).__init__()
		self.board = board
		
	def createPath(self, net):
		board = self.board
		start = board.gates[net.start_gate]
		end = board.gates[net.end_gate]

		curPos = start
		net.addPos(start)
		
		if curPos[2] < net.net_id:
			PHASE = 'UP'
		else: 
			PHASE = 'LAT'
		
		while not self.checkAdjacent(curPos, end):
			# if checkAdjacent(curPos, end):
			# 	return net

			dX = end[0] - curPos[0]
			dY = end[1] - curPos[1]

			# print dX, dY


			# print curPos[2], PHASE, net.net_id
			# UP PHASE
			if PHASE == 'UP':
				nextPos = self.goUp(curPos, board)
				if not nextPos:
					nextPos = self.goX(curPos, dX, board)
					if not nextPos:
						nextPos = self.goY(curPos, dY, board)
						if not nextPos:
							nextPos = self.goNotX(curPos, dX, board)
							if not nextPos:
								nextPos = self.goNotY(curPos, dY, board)
				if nextPos == False:
					net.path = False
					return net
				net.addPos(nextPos)
				board.setElementAt(net, nextPos[0], nextPos[1], nextPos[2])
				curPos = nextPos
				if curPos[2] == net.net_id:
					PHASE = 'LAT'
			# LATERAL PHASE
			if PHASE == 'LAT':
				nextPos = self.goX(curPos, dX, board)
				if not nextPos:
					nextPos = self.goY(curPos, dY, board)
					if not nextPos:
						nextPos = self.goNotX(curPos, dX, board)
						if not nextPos:
							nextPos = self.goNotY(curPos, dY, board)

				if nextPos == False:
					net.path = False
					return net
				net.addPos(nextPos)
				board.setElementAt(net, nextPos[0], nextPos[1], nextPos[2])
				curPos = nextPos
				if dX == 0 and dY == 0:
					PHASE = 'DOWN'
			# DOWN PHASE
			if PHASE == 'DOWN':
				nextPos = False
				if dX != 0:
					nextPos = self.goX(curPos, dX, board)
				elif dY != 0:
					nextPos = self.goY(curPos, dY, board)
				if not nextPos:	
					nextPos = self.goDown(curPos, board)
					if not nextPos:
						nextPos = self.goX(curPos, dX, board)
						if not nextPos:
							nextPos = self.goY(curPos, dY, board)
							if not nextPos:
								nextPos = self.goNotX(curPos, dX, board)
								if not nextPos:
									nextPos = self.goNotY(curPos, dY, board)
				if nextPos == False:
					net.path = False
					return net
				net.addPos(nextPos)
				board.setElementAt(net, nextPos[0], nextPos[1], nextPos[2])
				curPos = nextPos
				if curPos[2] == 0:
					PHASE = 'LAT'
		net.addPos(end)
		return net

	def checkAdjacent(self, curPos, end):
		# Same plane

		if curPos[2] == end[2]:

			if curPos[1] == end[1]:
				if abs(end[0] - curPos[0]) == 1:
					return True
			elif curPos[0] == end[0]:
				if abs(end[1] - curPos[1]) == 1:
					return True
		elif curPos[0] == end[0] and curPos[1] == end[1] and abs(end[2] - curPos[2]) == 1:
			return True
		else: return False

	def goUp(self, curPos, board):
		newPos = curPos[0], curPos[1], curPos[2] + 1
		if board.isEmpty(newPos):
			return newPos
		else:
			return False

	def goDown(self, curPos, board):
		newPos = curPos[0], curPos[1], curPos[2] - 1
		if board.isEmpty(newPos):
			return newPos
		else:
			return False

	def goX(self, curPos, dX, board):
		if dX != 0:
			newPos = curPos[0] + np.sign(dX), curPos[1], curPos[2]
		else:
			newPos = curPos[0] + 1, curPos[1], curPos[2]
		if board.isEmpty(newPos):
			return newPos
		else:
			return False

	def goY(self, curPos, dY, board):
		if dY != 0:
			newPos = curPos[0], curPos[1] + np.sign(dY), curPos[2]
		else:
			newPos = curPos[0], curPos[1] + 1, curPos[2]
		if board.isEmpty(newPos):
			return newPos
		else:
			return False

	def goNotX(self, curPos, dX, board):
		if dX != 0:
			newPos = curPos[0] - np.sign(dX), curPos[1], curPos[2]
		else:
			newPos = curPos[0] - 1, curPos[1], curPos[2]
		if board.isEmpty(newPos):
			return newPos
		else:
			return False

	def goNotY(self, curPos, dY, board):
		if dY != 0:
			newPos = curPos[0], curPos[1] - np.sign(dY), curPos[2]
		else:
			newPos = curPos[0], curPos[1] - 1, curPos[2]
		if board.isEmpty(newPos):
			return newPos
		else:
			return False	

	def goBack(self, curPos, board, net):
	# Doesn't work as expected
		if len(net.path) > 1:
			print '\ngoBack'
			print net.path[-1], len(net.path)
			net.path.remove(net.path[-1])
			print net.path[-1], len(net.path)
			board.removeElementAt(curPos)
			board.setElementAt(-1, curPos[0],curPos[1],curPos[2])
			prevPos = net.path[-1]
			print prevPos
			return prevPos
		return False