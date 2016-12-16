# -*- coding: utf-8 -*-
import random
import pprint
import numpy as np
import matplotlib.pyplot as plt
import math


class Sorter(object):

    """docstring for Sorter"""

    def __init__(self, netlist, board):
        super(Sorter, self).__init__()
        self.netlist = netlist
        self.board = board

    def cmpByDistance(self, a, b):
        a1, a2 = a
        b1, b2 = b
        (xa1,ya1,za1) = self.board.gates[a1].getCoordinates()
        (xa2,ya2,za2) = self.board.gates[a2].getCoordinates()
        (xb1,yb1,zb1) = self.board.gates[b1].getCoordinates()
        (xb2,yb2,zb2) = self.board.gates[b2].getCoordinates()
        adist = abs(xa1-xa2)+abs(ya1-ya2)+abs(za1-za2)
        bdist = abs(xb1-xb2)+abs(yb1-yb2)+abs(zb1-zb2)
        if adist > bdist:
            return 1
        elif adist == bdist:
            return 0
        else:
            return -1
    
    def cmpByAngle(self, a, b):
        a1, a2 = a
        b1, b2 = b
        (xa1,ya1,za1) = self.board.gates[a1].getCoordinates()
        (xa2,ya2,za2) = self.board.gates[a2].getCoordinates()
        (xb1,yb1,zb1) = self.board.gates[b1].getCoordinates()
        (xb2,yb2,zb2) = self.board.gates[b2].getCoordinates()
        aydiff = ya1-ya2
        axdiff = xa1-xa2
        bydiff = yb1-yb2
        bxdiff = xb1-xb2
        if axdiff != 0:
            aslope = aydiff/axdiff
        else:
            aslope = aydiff/0.0000001
        if bxdiff != 0:
            bslope = bydiff/bxdiff
        else:
            bslope = bydiff/0.0000001
        aAngle = math.degrees(math.atan(aslope))+90
        bAngle = math.degrees(math.atan(bslope))+90

    def sortNetlistByDistance(self):
        self.netlist.sort(self.cmpByDistance)
        return self.netlist

    def sortNetlistByAngle(self):
        self.netlist.sort(self.cmpByAngle)

    def random(self):
        random.shuffle(self.netlist)

    def cmpByDistance(self, a, b):
        a1, a2 = a
        b1, b2 = b
        (xa1, ya1, za1) = self.board.gates[a1]
        (xa2, ya2, za2) = self.board.gates[a2]
        (xb1, yb1, zb1) = self.board.gates[b1]
        (xb2, yb2, zb2) = self.board.gates[b2]
        adist = abs(xa1-xa2)+abs(ya1-ya2)+abs(za1-za2)
        bdist = abs(xb1-xb2)+abs(yb1-yb2)+abs(zb1-zb2)
        if adist > bdist:
            return 1
        elif adist == bdist:
            return 0
        else:
            return -1

    def cmpByAngle(self, a, b):
        a1, a2 = a
        b1, b2 = b
        (xa1, ya1, za1) = self.board.gates[a1]
        (xa2, ya2, za2) = self.board.gates[a2]
        (xb1, yb1, zb1) = self.board.gates[b1]
        (xb2, yb2, zb2) = self.board.gates[b2]
        aydiff = ya1-ya2
        axdiff = xa1-xa2
        bydiff = yb1-yb2
        bxdiff = xb1-xb2
        if axdiff != 0:
            aslope = aydiff/axdiff
        else:
            aslope = aydiff/0.0000001
        if bxdiff != 0:
            bslope = bydiff/bxdiff
        else:
            bslope = bydiff/0.0000001
        aAngle = math.degrees(math.atan(aslope))+90
        bAngle = math.degrees(math.atan(bslope))+90

        if aAngle > bAngle:
            return 1
        elif aAngle == bAngle:
            return 0
        else:
            return -1


if __name__ == '__main__':
    from helpers import *
    netlists = readNetlists()
    n = netlists[0]
    b = createBoard(0, 30)
    s = Sorter(n, b)
    s.sortNetlistByAngle()
    new_n = []
    for (a, b) in n:
        new_n.append((a+1, b+1))
    print new_n

class EasyPath(object):
	"""Attempt at creating a path, doesn't work!"""
	def __init__(self, board, net):
		super(EasyPath, self).__init__()
		self.board = board
		self.net = net
		
	def createPath(self):
		board = self.board
		start = board.gates[self.net.start_gate].getCoordinates()
		end = board.gates[self.net.end_gate].getCoordinates()

		curPos = start
		path = [start]
		
		if curPos[2] < self.net.net_id:
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
					path = []
					return path
				path.append(nextPos)
				board.setElementAt(self.net, nextPos[0], nextPos[1], nextPos[2])
				curPos = nextPos
				if curPos[2] == self.net.net_id:
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
					path = []
					return path
				path.append(nextPos)
				board.setElementAt(self.net, nextPos[0], nextPos[1], nextPos[2])
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
					path = []
					return path
				path.append(nextPos)
				board.setElementAt(self.net, nextPos[0], nextPos[1], nextPos[2])
				curPos = nextPos
				if curPos[2] == 0:
					PHASE = 'LAT'
		path.append(end)
		return path

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

class DepthFirst(object):
	"""Depthfirst search through netlist orderings in an attempt to find a solvable ordering.
	
	Args:
		board (:obj: Board): A board object on which to place the nets.
		netlist (list<tuples>): A list of tuples with start and end gates [(start1, end1), (start2, end2), ..., (startN, endN)]
	"""
	def __init__(self, board, netlist):
		super(DepthFirst, self).__init__()
		self.netlist = netlist
		self.board = board

	"""Iterates depth-first through netlist orderings, aided by sorting the nets in netlist (the heuristic).
	
	Returns:
		Board: A board with paths for all nets in netlist.
	"""
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
			print 'size of stack:',len(tree), ',nets to be solved:', len(currentNode.netlist)

			if len(currentNode.netlist) == 0:
				n = self.reconstructNetlist(currentNode)
				print 'Finding the right ordering took',iterations,'iterations' 
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
					net = Net(start, end, len(currentNode.netlist))

					#run the A* algorithm
					alg = AStar(new_board, net)
					path = alg.createPath()

					#if no path can be found, proceed to next net in netlist.
					if path == []:
						continue
					
					# add the net and path to the board
					net.path = path
					new_board.nets[net.net_id] = net
					new_board.setNetPath(net)

					# create new TreeNode object for current net
					new_node = TreeNode(new_board, currentNode, new_netlist)
					tree.append(new_node)
				
		# no solution found, practically unreachable due to the size of the tree
		return False

	def reconstructNetlist(self, currentNode):
		"""Reconstructs the netlist from a TreeNode

		Args:
			currentNode (:obj:Treenode): A node from which to traverse back to start.

		Returns:
			list<Tuple>: An ordered netlist
		"""
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
	"""
	BreadthFirst algorithm for finding a set of paths
	:param net: A Net object for which to find a path
	:param board: A Board object on which to plan the path
	"""
	def __init__(self, net, board):
		super(BreadthFirst, self).__init__()
		self.start = board.gates[net.start_gate]
		self.end = board.gates[net.end_gate]
		self.board = copy.deepcopy(board)
		(sx,sy,sz) = self.start.getCoordinates()
		(ex,ey,ez) = self.end.getCoordinates()
		self.max = abs(ex-sx)+abs(ey-sy)+abs(ez-sz)+2

	"""
	Creates at least all shortest paths, and maybe more. It runs for 10 percent more iterations than needed
	so that it finds some slightly longer paths as well.
	
	:return: A list of ordered lists of coordinates, paths between start and end of net. 
	"""
	def createPaths(self):
		queue = []
		visited = []
		dictPreviousNode = {}
		counter = 0
		maximum = -1
		# allPaths = []

		queue.insert(0, self.start.getCoordinates())
		while len(queue) != 0:
			currentNode = queue.pop()
			if currentNode not in visited:
				# vind alle buren
				(x,y,z) = currentNode
				(sx,sy,sz) = self.start.getCoordinates()
				currentDepth = abs(x-sx)+abs(y-sy)+abs(z-sz)

				# If currentNode is adjacent to the end node, we are almost done!
				if self.end.getCoordinates() in self.board.getAllNeighbours(x,y,z):
					if self.end.getCoordinates() not in dictPreviousNode.keys():
						dictPreviousNode[self.end.getCoordinates()] = [currentNode]
					else:
						dictPreviousNode[self.end.getCoordinates()].append(currentNode)
					# return self.reconstructPaths(dictPreviousNode, [self.end.getCoordinates()])
					
				if currentDepth > self.max:
					allPaths = self.reconstructPaths(dictPreviousNode, [self.end.getCoordinates()],[])
					return allPaths

				# voeg buren toe aan queue als je deze nog niet gecheckt hebt
				neighbours = self.board.getOpenNeighbours(x,y,z)
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
		allPaths = self.reconstructPaths(dictPreviousNode, [self.end.getCoordinates()],[])
		return allPaths

	def reconstructPaths(self, cameFrom, path, paths=[]):
		"""Reconstructs all paths between goal and start node

		Args:
			cameFrom (dict): a dictionary that maps each coordinate to a list of coordinates from which this can be reached
			path (list): a path
			paths (list): a list of paths

		Return:
			list: a list of all paths.		
		"""
		lastNode = path[-1]
		if lastNode in cameFrom.keys():
			for nextNode in cameFrom[lastNode]:
				new_path = path + [nextNode]
				paths = self.reconstructPaths(cameFrom, new_path, paths)
		else:
			path = list(reversed(path))
			paths += [path]
		return paths


class HillClimber(object):
	"""Hillclimber Algorithm NOT FINISHED, replaced halfway through by parallel hillclimber (or geneticOpt)
	"""
	def __init__(self, board, iterations=1):
		super(HillClimber, self).__init__()
		self.board = board
		self.iterations = iterations

	def solve(self):

		for i in range(self.iterations):
			
			print 'hillclimbing iteration:', i
			random_net = random.choice(self.board.nets)
			print 'changing net:', random_net.net_id
			print 'With current path:', random_net.path
			oldpath = copy.deepcopy(random_net.path)

			startGateID = self.board.getElementAt(random_net.start_gate[0],random_net.start_gate[1],random_net.start_gate[2]).gate_id
			endGateID = self.board.getElementAt(random_net.end_gate[0],random_net.end_gate[1],random_net.end_gate[2]).gate_id
			
			if not self.board.removeNetPath(random_net):
				print 'Failed removing old path!'

			breadthFirstAlgorithm = BreadthFirst(startGateID, endGateID, self.board)
			possibleNewPaths = breadthFirstAlgorithm.solve()

			print 'found', len(possibleNewPaths), 'paths'
			if len(possibleNewPaths) < 1:
				continue
			
			random_net.path = random.choice(possibleNewPaths)
			print 'New path is:', random_net.path

			# astar = AStar(self.board, random_net)
			# print random_net.path
			# random_net = astar.createPath(random_net.start_gate, random_net.end_gate, 'no_bias')
			# print random_net.path


			if len(random_net.path) <= len(oldpath):
				print 'selected path is smaller than or equal to original path'
				if not self.board.setNetPath(random_net):
					print 'Failed placing path!'
					break
			else:
				random_net.path = oldpath
				if not self.board.setNetPath(random_net):
					print 'Failed restoring old path!'
					break

		return self.board