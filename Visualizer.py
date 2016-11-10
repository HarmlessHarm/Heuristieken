import matplotlib as mpl 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import axes3d
import numpy as np

class Visualizer(object):


	def start(self):
		fig = plt.figure()
		ax = fig.add_subplot(111,projection='3d')
		xs,ys,zs = self.createIntersections()
		self.plotIntersections(xs, ys, zs, ax)
		ax.set_xlabel('width')
		ax.set_ylabel('height')
		ax.set_zlabel('layer')
		plt.axis('off')
		plt.show()

	def createIntersections(self):
		dims = self.board.getDimensions()
		intersectionCount = dims[0] * dims[1]
		xs = range(dims[0])*dims[1]
		ys = range(dims[1])*dims[0]
		zs = [0]*intersectionCount
		return xs,ys,zs
		
	def plotIntersections(self, xs, ys, zs, ax):
		for i in range(len(xs)):
			elem = self.board.getElementAt(xs[i],ys[i],zs[i])
			if  elem == 0:
				ax.scatter(xs[i],ys[i],zs[i], c='w', marker='o')
			else:
				ax.scatter(xs[i],ys[i],zs[i], c='r', marker='o', label=str(elem))
		

