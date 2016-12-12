# -*- coding: utf-8 -*-
import matplotlib as mpl 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np

class Visualizer(object):

	def __init__(self, board, board2 = False):
		self.board = board
		self.board2 = board2

	def start(self):
		fig = plt.figure(figsize=(15, 8))
		# title centered above all subplots
		st = fig.suptitle("Boards", fontsize = "x-large")

		if self.board2 is False:
			ax1 = fig.add_subplot(111, projection = '3d')
		else:
			ax1 = fig.add_subplot(121, projection = '3d')

		ax1.set_zlim([0,self.board.z_dim])
		# ax1.set_title("Absolute score: " + str(self.board.getScore()[1]) + "\n Relative score: " + str(self.board.getRelativeScore()))

		# self.plotGridLines(ax1)
		self.plotGates(ax1, self.board)
		self.plotNets(ax1, self.board)
		ax1.set_xlabel('width')
		ax1.set_ylabel('height')
		ax1.set_zlabel('layer')
		# plt.axis('off')

		if self.board2 is not False:
			ax2 = fig.add_subplot(122, projection = '3d')
			ax2.set_zlim([0,self.board2.z_dim])
			ax2.set_title("Absolute score: " + str(self.board2.getScore()[1]) + "\n Relative score: " + str(self.board2.getRelativeScore()))

			# self.plotGridLines(ax2)
			self.plotGates(ax2, self.board2)
			self.plotNets(ax2, self.board2)
			ax2.set_xlabel('width')
			ax2.set_ylabel('height')
			ax2.set_zlabel('layer')
			# plt.axis('off')

		plt.show()

	def plotGates(self, ax, board):
		#print self.board.gates
		for g in board.gates:
			x,y,z = board.gates[g].getCoordinates()
			ax.scatter(x,y,z, c='r', marker='s')
			ax.text(x,y,z,str(g),color='k', fontsize=12)

	def plotGridLines(self, ax):
		width, height, depth = self.board.getDimensions()	
		for i in range(width+1):
			ax.plot([i,i],[0,height], c='0.2')
		for j in range(height+1):
			ax.plot([width, 0], [j,j], c='0.2')
	
	def plotNets(self, ax, board):
		nets = board.nets
		cmap = plt.get_cmap('gnuplot')
		colors = [cmap(i) for i in np.linspace(0, 1, len(nets))]
		for net_id, color in zip(nets.keys(),colors):
			net = nets[net_id]
			if net.path != False:
				for i, (x,y,z) in enumerate(net.path[:-1]):
					(nx,ny,nz) = net.path[i+1]
					ax.plot([x,nx],[y,ny],[z,nz], color=color, alpha=0.8)

	
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