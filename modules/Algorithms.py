import numpy as np
from random import shuffle
import sys
from Objects import *
from Visualizer import *
from Sorter import *
import copy

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
	# Does not work as expected
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
	
	def createPath(self, start, goal, bias='vertical'):
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
		fScore[x_start][y_start][z_start] = self.manhattanCostEstimate(start, goal)

		while openSet != []:
			# Set currentNode to be the node in openset with the lowest fscore (above -1)
			(cx,cy,cz) = openSet[0]
			for (x,y,z) in openSet:
				if  0 <= fScore[x][y][z] < fScore[cx][cy][cz]:
					(cx,cy,cz) = (x,y,z)

			# if currentNode is adjacent to goal node, return the path to currentNode
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

				tentative_gscore = gScore[cx][cy][cz] + self.distance((nx,ny,nz), bias)
				if not (nx,ny,nz) in openSet:
					openSet.append((nx,ny,nz))
				elif tentative_gscore >= gScore[nx][ny][nz] >= 0:
					continue

				cameFrom[(nx,ny,nz)] = (cx,cy,cz)
				gScore[nx][ny][nz] = tentative_gscore
				fScore[nx][ny][nz] = gScore[nx][ny][nz] + self.manhattanCostEstimate((nx,ny,nz),goal)
		self.net.path = False
		return self.net

	# Distance to node, based on its neighbours and the layer it is in
	def distance(self, node, bias):
		(x,y,z) = node
		distance = 1
		if bias=='vertical':
			for (nx,ny,nz) in self.board.getAllNeighbours(x,y,z):
				if type(self.board.getElementAt(nx,ny,nz)) is Gate:
					distance += 4 #should be just enough to make the path that leaves one space around a gate be cheaper than the path that doesn't
				elif type(self.board.getElementAt(nx,ny,nz)) is Net:
					distance += 3 #Add one distance for every adjacent net, this should space things out a bit

				#Make higher paths more attractive
				distance += pow(self.board.z_dim, 2) / (nz+1)
		
		elif bias=='lateral':
			for (nx,ny,nz) in self.board.getAllNeighbours(x,y,z):
				if type(self.board.getElementAt(nx,ny,nz)) is Gate:
					distance += 4 #should be just enough to make the path that leaves one space around a gate be cheaper than the path that doesn't
				elif type(self.board.getElementAt(nx,ny,nz)) is Net:
					distance += 3 #Add one distance for every adjacent net, this should space things out a bit

				#Make paths that are higher and avoid the board center more attractive
				xCenter = self.board.x_dim/2
				yCenter = self.board.y_dim/2
				distanceFromCenter = abs(nx-xCenter)+abs(ny-yCenter)
				distance += min(xCenter, yCenter) / (distanceFromCenter+1)
				distance += pow(self.board.z_dim, 2) / (nz+1)
	

		return distance


	# Very optimistic heuristic, it returns the manhattan distance between the 2 nodes
	def manhattanCostEstimate(self, node1, node2):
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
	"""docstring for Depthfirst"""
	def __init__(self, netlist, board):
		super(DepthFirst, self).__init__()
		self.netlist = netlist
		self.board = board

	def solve(self):
		tree = []
		discovered = []

		# create TreeNode object for root
		start_node = TreeNode(self.board, 'start', self.netlist)
		tree.append(start_node)
		iterations = 0

		while len(tree) != 0:
			currentNode = tree.pop()
			iterations += 1
			print 'size of stack:',len(tree), 'nets to be solved:', len(currentNode.netlist)

			if len(currentNode.netlist) == 0:
				n = self.reconstructNetlist(currentNode)
				print 'Finding the answer took',iterations,'iterations' 
				return currentNode.board, n

			if currentNode not in discovered:
				discovered.append(currentNode)

				# sort netlist by distance (low - high)
				s = Sorter(currentNode.netlist, currentNode.board)
				currentNode.netlist = s.sortNetlistByDistance()

				# check every net in netlist
				for (start, end) in currentNode.netlist:
					new_board = copy.deepcopy(currentNode.board)
					new_netlist = copy.deepcopy(currentNode.netlist)
					new_netlist.remove((start, end))

					# create net object for current net
					net = Net(new_board.gates[start], new_board.gates[end], len(currentNode.netlist))
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

	# every element
	def reconstructNetlist(self, currentNode):
		netlist = []
		lastNode = currentNode.previousNode
		while lastNode != 'start':
			#Het laatst opgeloste net is het verschil tussen de overgebleven netlist van currentnode en lastnode
			net = set(lastNode.netlist).symmetric_difference(currentNode.netlist).pop()
			netlist.append(net)
			currentNode = lastNode
			lastNode = currentNode.previousNode
		reversed(netlist)
		return netlist

class BreadthFirst(object):
	"""docstring for BreadthFirst"""
	def __init__(self, start, end, board):
		super(BreadthFirst, self).__init__()
		self.start = board.gates[start]
		self.end = board.gates[end]
		self.board = board


	def solve(self):
		queue = []
		visited = []
		dictPreviousNode = {}
		counter = 0
		maximum = -1

		queue.insert(0, self.start)
		while len(queue) != 0:
			currentNode = queue.pop()

			if currentNode not in visited:
				# vind alle buren
				(x,y,z) = currentNode

				# If currentNode is adjacent to the end node, we are done!
				if self.end in self.board.getAllNeighbours(x,y,z):
					
					if self.end not in dictPreviousNode.keys():
						dictPreviousNode[self.end] = [currentNode]
					else:
						dictPreviousNode[self.end].append(currentNode)
					#return self.reconstructPaths(dictPreviousNode, [self.end])
					#maximum = counter * 3
					
				#if counter > maximum and maximum > 0:
				#	return self.reconstructPaths(dictPreviousNode, [self.end])


				neighbours = self.board.getOpenNeighbours(x,y,z)
				# voeg buren toe aan queue als je deze nog niet gecheckt hebt

				for neighbour in neighbours:
					if neighbour in visited:
						continue

					if neighbour not in dictPreviousNode.keys():
						dictPreviousNode[neighbour] = [currentNode]
					else:
						dictPreviousNode[neighbour].append(currentNode)

					queue.insert(0, neighbour)

				visited.append(currentNode)
				counter += 1
		# lala
		return self.reconstructPaths(dictPreviousNode, [self.end])

	def reconstructPaths(self, cameFrom, path, paths = []):
		lastNode = path[-1]
		if lastNode in cameFrom.keys():
			for nextNode in cameFrom[lastNode]:
				new_path = path + [nextNode]
				paths = self.reconstructPaths(cameFrom, new_path, paths)
		else:
			paths += [path]
		return paths

class GeneticOpt(object):
	"""docstring for GeneticOpt"""
	def __init__(self, base_board, max_generations, max_population):
		super(GeneticOpt, self).__init__()
		self.base_board = base_board
		self.max_generations = max_generations
		self.max_population = max_population
		self.base_score = self.base_board.getScore()[1]
		self.population = self.initPop()

	def initPop(self):

		pop = []
		for i in range(self.max_population):
			pop.append((copy.deepcopy(self.base_board), self.base_score))
		return pop

	def run(self):
		for i in range(self.max_generations):
			print "In Generation", i
			newPop = self.iteration(self.population)
			sortedPop = self.sortPop(newPop)
			killedPop = self.killPop(sortedPop)
			self.population = self.repopulate(killedPop)
		print "Improved from", self.base_score, 'to', self.population[0][1]
		oldV  = Visualizer(self.base_board)
		oldV.start()
		v = Visualizer(self.population[0][0])
		v.start()
		

	def iteration(self, population):
		newPop = []
		for board, score in population:
			net = random.choice(board.nets)
			oldPath = net.path
			board.removeNetPath(net)
			# net = Net(ran/d_net[0], net[1], i)
			astar = AStar(board, net)
			net = astar.createPath(net.start_gate, net.end_gate, bias=False)
			if not net.path:
				print '\nFailed planning a better path for net', i, '!'
				net.path = oldPath
				board.setNetPath(net)
			else:
				board.setNetPath(net)
				newScore = board.getScore()[1]
				if newScore == score:
					pass
					# print "No improvement, cost:", score
				else:
					print "Found new path: old score:", score, "new score:", newScore
				newPop.append((board, newScore))
		return newPop

	def sortPop(self, pop):
		return sorted(pop, key=lambda tup:tup[1])

	def killPop(self, pop):
		return pop[:len(pop)/2]

	def repopulate(self, pop):
		return copy.deepcopy(pop) + copy.deepcopy(pop)


class HillClimber(object):
	"""docstring for hillClimber"""
	def __init__(self, board):
		super(HillClimber, self).__init__()
		self.board = board
		
	def climb(self, iterations=1):

		for i in range(iterations):
			print 'hillclimbing iteration:', i
			random_net = random.choice(self.board.nets)
			oldpath = copy.copy(random_net.path)
			self.board.removeNetPath(random_net)
			startGateID = self.board.getElementAt(random_net.start_gate[0],random_net.start_gate[1],random_net.start_gate[2]).gate_id
			endGateID = self.board.getElementAt(random_net.end_gate[0],random_net.end_gate[1],random_net.end_gate[2]).gate_id
			
			breadthFirstAlgorithm = BreadthFirst(startGateID, endGateID, self.board)
			possibleNewPaths = breadthFirstAlgorithm.solve()
			print 'found', len(possibleNewPaths), 'paths'
			newPath = random.choice(possibleNewPaths)
			if len(newPath) <= len(oldpath):
				print 'selectedpath is smaller than or equal to original path'
				random_net.path = newPath
				if not self.board.setNetPath(random_net):
					print 'Failed placing path!'
					break

		return self.board

if __name__ == '__main__':
	from helpers import *

	netlist = [(15, 8), (3, 15), (15, 5), (20, 19), (23, 4), (5, 7), (1, 0), (15, 21), (3, 5), (7, 13), (3, 23), (23, 8), (22, 13), (15, 17), (20, 10), (13, 18), (19, 2), (22, 11), (10, 4), (11, 24), (2, 20), (3, 4), (16, 9), (19, 5), (3, 0), (6, 14), (7, 9), (9, 13), (22, 16), (10, 7)]
	board = runAlgorithm('astar', 0, netlist, 10, recursive=True)
	print '\nold board score:', board.getScore()
	hc = HillClimber(board)
	newBoard = hc.climb(20)
	print 'new board score:', newBoard.getScore()
	v = Visualizer(newBoard)
	v.start()