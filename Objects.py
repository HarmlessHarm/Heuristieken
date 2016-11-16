import pprint
import numpy as np


class Board(object):

	"""docstring for Board"""

	def __init__(self, x_dim, y_dim, z_dim=1):
		super(Board, self).__init__()
		self.x_dim = x_dim
		self.y_dim = y_dim
		self.z_dim = z_dim

		self.board = np.zeros((x_dim, y_dim, z_dim), dtype=object)

		self.gates = {}
		self.nets = {}

	def getElementAt(self, x, y, z):
		if  x > self.x_dim - 1 or y > self.y_dim - 1 or z > self.z_dim - 1:
			return True
		return self.board[x, y, z]

	def isEmpty(self, xyz):
		x, y, z = xyz
		if self.getElementAt(x, y, z) == 0:
			return True
		else:
			return False

	def setElementAt(self, obj,  x, y, z=0):
		if self.isEmpty((x,y,z)):
			self.board[x][y][z] = obj
			return True
		else:
			return False

	def addLayer(self):
		x, y = (self.x_dim, self.y_dim)
		layer = np.zeros((x,y,1))
		self.z_dim += 1
		self.board = np.append(self.board, layer, axis=2)

	def getLayer(self, layer):
		return self.board[:, :, layer]

	def getDimensions(self):
		return (self.x_dim, self.y_dim, self.z_dim)

	def getScore(self):
		score = 0
		for net in self.nets:
			score += len(self.nets[net].path)
		return score
		
	def printBoard(self):
		for z in range(self.z_dim):
			print 'layer',z
			print self.getLayer(z)
			print "\n"


class Gate(object):

	""" Docstring for Gate """

	def __init__(self, gate_id, x, y, z=0):
		self.gate_id = gate_id
		self.x = x
		self.y = y
		self.z = z

	def getCoordinates(self, coordinates):
		return (x, y, z)


class Net:

#   """ Docstring for Net """

	def __init__(self, start_gate, end_gate, net_id):
		self.net_id = net_id
		self.start_gate = start_gate
		self.end_gate = end_gate
		self.path = []

	def addPos(self,xyz):
	 	x, y ,z = xyz
		self.path.append((x,y,z))

if __name__ == '__main__':
	b = Board(3, 3)
	b.printBoard()
	print "add"
	b.addLayer()
	print 'added'
	print b.board.shape[2]