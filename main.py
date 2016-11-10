from Objects import Board, Gate, Net
from Algorithms import EasyPath
# from Visualizer import Visualizer
import numpy as np
import json


def createBoard():

	file = 'prints.json'

	with open(file) as json_data:

	    d = json.load(json_data)
	    board = d['prints'][2]
	    width = board['width']
	    height = board['height']
	    b = Board(width, height)
	    for gate in board['gates']:
	    	g = Gate(gate['x'], gate['y'], gate['id'])
	    	b.gates[g.gate_id] = (g.x, g.y)
	        b.setElementAt(g.gate_id, g.x, g.y)

	    print b.getLayer(0)
	    return b
	    # v = Visualizer(b)
	    # v.start()

def getPath(start_gate, end_gate, board):
	
	net = Net(start_gate, end_gate)

	start = board.gates[start_gate]
	end = board.gates[end_gate]
	curPos = (start[0],start[1],0)
	dX = end[0] - start[0]
	dY = end[1] - start[1]



	# while curPos != end:
	#	dX = end[0] - curPos[0]
	#	dY = end[1] - curPos[1]
#
#		nextPos = getNextPos(curPos, end, dX, dY, board)
#		net.addPos(nextPos)
#		curPos = nextPos

#	return net

def getNextPos(curPos, end, dX, dY, board):
	# Check if voxel below is empty
	if curPos[2] != 0:
		print curPos
		downPos = curPos[0], curPos[1], curPos[2] - 1
		if board.isEmpty(downPos):
			return (downPos)

	if dX != 0:
		nextPos = (curPos[0] + np.sign(dX),curPos[1], curPos[2])
		if board.isEmpty(nextPos):
			return nextPos
		else:
			return (curPos[0], curPos[1], curPos[2] + 1)
	elif dY != 0:
		nextPos = (curPos[0],curPos[1] + np.sign(dY), curPos[2])
		if board.isEmpty(nextPos):
			return nextPos
		else:
			return (curPos[0], curPos[1], curPos[2] + 1)

	else:
		print 'huh?'

	


if __name__ == '__main__':
	board = createBoard()
	netList = [(1,4),(2,3)]
	
	for (start,end) in netList:
		net = getPath(start, end, board)
		print net.path
