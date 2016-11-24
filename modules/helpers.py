import json, ast, os
import numpy as np
from Objects import *
from Algorithms import *

def readNetlists():

	path = os.path.dirname(os.path.abspath(__file__))
	netlist_file = path + '/../resources/netlist.txt'
	net_lists = []
	with open(netlist_file, 'r') as netlist_data:
	    nd = netlist_data.readlines()
	    for line in nd:
	        if line[0] == 'n':
	            line = line.split(' = ')
	            net_lists.append(ast.literal_eval(line[1]))
	        else:
	            continue

	return net_lists
		
def createBoard(i, layers):

	path = os.path.dirname(os.path.abspath(__file__))
	file = path + '/../resources/prints.json'

	with open(file) as json_data:

	    d = json.load(json_data)
	    board = d['prints'][i]
	    width = board['width']
	    height = board['height']
	    b = Board(width, height, layers)
	    for gate in board['gates']:
	    	g = Gate(gate['id']-1, gate['x'], gate['y'])
	    	b.gates[g.gate_id] = (g.x, g.y, g.z)
	        b.setElementAt(g, g.x, g.y)

	    return b

def runAlgorithm(alg_str, board, netlist, suppress=False):
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

			# if failedCount > 0:
				
			if not net.path:
					if not suppress: print 'Failed planning a path for net', i, '!'
					failedCount += 1
					print "Failed"
					return False
					# continue
			else:
				board.nets[net.net_id]=net
			#print 'about to set this planned path: ', plannedPath
			if not board.setNetPath(net):
				if not suppress: print 'Path is planned over an occupied position, something went seriously wrong!'
				break
	print 'Failed planning paths for: ', failedCount, 'nets'
	return True
