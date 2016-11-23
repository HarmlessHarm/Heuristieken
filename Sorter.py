from main import *


class Sorter(object):
	"""docstring for Sorter"""
	def __init__(self, netlist, board):
		super(Sorter, self).__init__()
		self.netlist = netlist
		self.board = board
		
	def sortNetlistByDistance(self):
		self.netlist.sort(self.cmpByDistance)
		return self.netlist
		

	def cmpByDistance(self, (a1, a2), (b1,b2)):
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

	# def cmpByCardinalDirection(self, (a1, a2), (b1,b2)):
	# 	(xa1,ya1,za1) = self.board.gates[a1]
	# 	(xa2,ya2,za2) = self.board.gates[a2]
	# 	(xb1,yb1,zb1) = self.board.gates[b1]
	# 	(xb2,yb2,zb2) = self.board.gates[b2]
	# 	pass


if __name__ == '__main__':
	b = createBoard(0,1)
	netlists = readNetlists()
	n = netlists[0]
	s = Sorter(n,b)
	print s.sortNetlistByDistance()

	