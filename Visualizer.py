import matplotlib as mpl 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from Objects import *
from Algorithms import *
from main import *
import numpy as np

class Visualizer(object):

	def __init__(self, board):
		self.board = board

	def start(self):
		fig = plt.figure()
		ax = fig.add_subplot(111,projection='3d')
		#self.plotGridLines(ax)
		self.plotGates(ax)
		self.plotNets(ax)
		ax.set_xlabel('width')
		ax.set_ylabel('height')
		ax.set_zlabel('layer')
		#plt.axis('off')
		plt.show()

	def plotGates(self, ax):
		#print self.board.gates
		for g in self.board.gates:
			x,y,z = self.board.gates[g]
			ax.scatter(x,y,z, c='r', marker='s')

	def plotGridLines(self, ax):
		width, height, depth = self.board.getDimensions()	
		for i in range(width+1):
			ax.plot([i,i],[0,height], c='0.2')
		for j in range(height+1):
			ax.plot([width, 0], [j,j], c='0.2')
	
	def plotNets(self, ax):
		nets = self.board.nets
		cmap = plt.get_cmap('gnuplot')
		colors = [cmap(i) for i in np.linspace(0, 1, len(nets))]
		for net_id, color in zip(nets.keys(),colors):
			net = nets[net_id]
			for i, (x,y,z) in enumerate(net.path[:-1]):
				(nx,ny,nz) = net.path[i+1]
				ax.plot([x,nx],[y,ny],[z,nz], color=color)

	
if __name__ == '__main__':
	# b = createBoard(1)
	# alg = AStar(b)
	# net = Net(0, 1, 0)
	# path = alg.aStar(b.gates[0],b.gates[1])
	# net.path = path
	# b.nets[0] = net
	# b.setNetPath(net)
	# v = Visualizer(b)
	# v.start()
	netlists = readNetlists()
	board = createBoard(30)
	failedCount = 0
	netlist = netlists[0]
	#shuffle(netlist)
	for i, (start, end) in enumerate(netlist):
		net = Net(board.gates[start], board.gates[end], 1)
		print "Planning the path", i, "from gate ", start, board.gates[start], " to gate ", end, board.gates[end]
		alg = AStar(board)
		plannedPath = alg.aStar(net.start_gate, net.end_gate)
		if not plannedPath:
			print 'Failed planning a path for net', i, '!'
			failedCount += 1
			continue

		net.path = plannedPath
		board.nets[i] = net
		if not board.setNetPath(net):
			print 'Path is planned over an occupied position!'
			break
	print 'Failed planning paths for: ', failedCount, 'nets'
	v = Visualizer(board)
	v.start()