import random, pprint
import numpy as np
import matplotlib.pyplot as plt

class Sorter(object):
	"""docstring for Sorter"""
	def __init__(self, netlist, board):
		super(Sorter, self).__init__()
		self.netlist = netlist
		self.board = board
		
	def sortNetlistByDistance(self):
		self.netlist.sort(self.cmpByDistance)
		return self.netlist
		

	def cmpByDistance(self, a, b):
		a1, a2 = a
		b1, b2 = b
		(xa1,ya1,za1) = self.board.gates[a1]
		(xa2,ya2,za2) = self.board.gates[a2]
		(xb1,yb1,zb1) = self.board.gates[b1]
		(xb2,yb2,zb2) = self.board.gates[b2]
		adist = abs(xa1-xa2)+abs(ya1-ya2)+abs(za1-za2)
		bdist = abs(xb1-xb2)+abs(yb1-yb2)+abs(zb1-zb2)
		if adist > bdist:
			return 1
		elif adist == bdist:
			return 0
		else:
			return -1

	def random(self):
		random.shuffle(self.netlist)


if __name__ == '__main__':
	from helpers import *
	netlists = readNetlists()
	n = netlists[0]
	l = []
	validLists = []
	for i in range(50):
		print i,
		b = createBoard(0, 30)
		s = Sorter(n,b)
		s.random()
		if runAlgorithm('astar', s.board, s.netlist, True):
			validLists.append(s.netlist)
			solved, score = s.board.getScore()
			l.append(score)
	# a = np.histogram(l)
	# print a
	pprint.pprint(validLists)
	print l
	plt.hist(l)
	plt.show()