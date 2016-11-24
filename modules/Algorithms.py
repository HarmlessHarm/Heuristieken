import numpy as np
from main import *
from random import shuffle
import sys
from Objects import *
import copy

def runAlgorithm(alg_str, board, netlist):
	failedCount = 0
	for i, (start, end) in enumerate(netlist):
			#print "Planning path for net", i, "from gate ", start, board.gates[start], " to gate ", end, board.gates[end]
			net = None
			if alg_str=='astar':
				net = Net(board.gates[start], board.gates[end], i)
				alg = AStar(board, net)
				net = alg.createPath(net.start_gate, net.end_gate)
			elif alg_str=='dijkstra':
				net = Net(start, end, i)
				alg = Dijkstra(board, net)
				net = alg.createPath()
			elif alg_str=='simple':
				net = Net(start, end, i)
				alg = EasyPath(board)
				net = alg.createPath(net)
				board.removeNetPath(net)

			if not net.path:
					print 'Failed planning a path for net', i, '!'
					failedCount += 1
					continue
			else:
				board.nets[net.net_id]=net
			#print 'about to set this planned path: ', plannedPath
			if not board.setNetPath(net):
				print 'Path is planned over an occupied position, something went seriously wrong!'
				break
	print 'Failed planning paths for: ', failedCount, 'nets'

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

class AStar(object):
	"""docstring for AStar"""
	def __init__(self, board, net):
		super(AStar, self).__init__()
		self.board = board	
		self.net = net
	
	def createPath(self, start, goal):
		(x,y,z) = self.board.getDimensions()
		(x_start, y_start, z_start) = start
		#For each node, whether it has been evaluated
		closedSet = np.zeros((x,y,z), dtype=bool)

		#Set of discovered nodes that still need evaluation, where a node is a tuple (x,y,z)
		openSet = [start]

		#Dictionary that for each node (a tuple x,y,z) archives which node it can most easily be reached from
		cameFrom = {}

		#Array that contains g_scores for all nodes, the cost of getting from start to that node
		#default is -1, as placeholder for proper default infinity
		gScore = np.full((x,y,z), -1, dtype='int64')
		gScore[x_start][y_start][z_start] = 0 #Cost of going from start to start is 0

		#array that contains f_scores for all nodes, the distance for getting to the goal node from start via that node
		#Default is -1, as placeholder for infinity
		fScore = np.full((x,y,z), -1, dtype='int64')
		fScore[x_start][y_start][z_start] = self.costEstimate(start, goal)

		while openSet != []:
			#Set currentNode to be the node in openset with the lowest fscore (above -1)
			(cx,cy,cz) = openSet[0]
			for (x,y,z) in openSet:
				if  0 <= fScore[x][y][z] < fScore[cx][cy][cz]:
					(cx,cy,cz) = (x,y,z)

			#if currentNode is adjacent to goal node, return the path to currentNode
			if goal in self.board.getAllNeighbours(cx,cy,cz):
				cameFrom[goal] = (cx,cy,cz)
				path = self.reconstructPath(cameFrom, goal)
				self.net.path = path
				return self.net

			openSet.remove((cx, cy, cz))
			closedSet[cx][cy][cz] = 1
			
			for (nx,ny,nz) in self.board.getOpenNeighbours(cx,cy,cz):
				if closedSet[nx][ny][nz]:
					continue #neighbour is already evaluated

				tentative_gscore = gScore[cx][cy][cz] + self.distance((nx,ny,nz))
				if not (nx,ny,nz) in openSet:
					openSet.append((nx,ny,nz))
				elif tentative_gscore >= gScore[nx][ny][nz] >= 0:
					continue

				cameFrom[(nx,ny,nz)] = (cx,cy,cz)
				gScore[nx][ny][nz] = tentative_gscore
				fScore[nx][ny][nz] = gScore[nx][ny][nz] + self.costEstimate((nx,ny,nz),goal)
		self.net.path = False
		return self.net

	def distance(self, node):
		(x,y,z) = node
		distance = 1
		for (nx,ny,nz) in self.board.getAllNeighbours(x,y,z):
			if type(self.board.getElementAt(nx,ny,nz)) is Gate:
				distance += 4 #should be just enough to make the path that leaves one space around a gate be cheaper than the path that doesn't
		return distance

	#Very optimistic heuristic, it returns the manhattan distance between the 2 nodes
	def costEstimate(self, node1, node2):
		(x,y,z) = node1
		(x2,y2,z2) = node2
		return abs(x2-x)+abs(y2-y)+abs(z2-z)

	def reconstructPath(self, cameFrom, currentNode):
		path = [currentNode]
		while currentNode in cameFrom.keys():
			currentNode = cameFrom[currentNode]
			path.append(currentNode)
		return list(reversed(path))

class Dijkstra(object):
	"""docstring for Dijkstra"""
	def __init__(self, board, net):
		super(Dijkstra, self).__init__()
		self.board = board
		self.net = net
		self.remaining = {}
		self.explored = {}

	def createPath(self):

		# define start/end gate
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
			coord = nextCoord

		return self.net


	def explore(self, coord, val):

		newRemaining = {}

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
		if len(neighbours) == 0:
			return False

		for neighbour in neighbours:
			if neighbour not in self.explored:
				# increase neighbour values based on current pos
				newRemaining[neighbour] = val + 1

		return newRemaining


	def getLowestValue(self, coord):

		neighbours = self.board.getOpenNeighbours(coord[0], coord[1], coord[2])
		if len(neighbours) == 0:
			return False

		# set max value
		lowestValue = sys.maxint
		
		for neighbour in neighbours:
			if neighbour in self.explored:
				# find neighbour value
				value = self.explored[neighbour]
				# if current neighbour value is lowest till now
				if value < lowestValue:
					lowestValue = value
					bestCoord = neighbour

		return bestCoord


class DepthFirst(object):
	"""docstring for ClassName"""
	def __init__(self, netlist, board):
		super(DepthFirst, self).__init__()
		self.netlist = netlist
		self.board = board

	def solve(self):
		tree = []
		discovered = []

		# create TreeNode object for root
		start_node = TreeNode(self.board, "start", self.netlist)

		tree.append(start_node)


		while len(tree) != 0:
			currentNode = tree.pop()
			if len(currentNode.netlist) == 0:
				# solution is found
				return currentNode.board

			if currentNode not in discovered:
				discovered.append(currentNode)

				# check every net in netlist
				for (start, end) in currentNode.netlist:
					new_board = copy.copy(currentNode.board)
					new_netlist = copy.copy(currentNode.netlist)
					new_netlist.remove((start, end))

					# create net object for current net
					net = Net(new_board.gates[start], new_board.gates[end], len(currentNode))
					alg = AStar(new_board, net)
					net = alg.createPath(net.start_gate, net.end_gate)

					if not net.path:
						continue

					new_board.nets[net.net_id] = net
					# add path to board
					new_board.setNetPath(net)

					# create new TreeNode object for current net
					new_node = TreeNode(new_board, currentNode, new_netlist)

					tree.append(new_node)
		# no solution found
		return False