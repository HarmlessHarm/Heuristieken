import json, ast, os
import numpy as np
from Objects import *
from Algorithms import *
import sys

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

def runAlgorithm(alg_str, board_number, netlist, maxLayers, recursive=True):
	failedCount = 0
	board = createBoard(board_number, maxLayers)
	#print 'call runAlgorithm with', board
	i = checkNetlist(alg_str, board, netlist)
	if not i is True and recursive:
		newNetlist = [netlist[i]] + netlist[:i] + netlist[i+1:]
		board = runAlgorithm(alg_str, board_number, newNetlist, maxLayers)
	return board

def checkNetlist(alg_str, board, netlist):
	failedCount = 0
	for i, (start, end) in enumerate(netlist):
		if not checkPath(alg_str, board, start, end, i):
			print netlist
			return i
	return True

def checkPath(alg_str, board, start, end, i):
	net = None
	print '.',
	sys.stdout.flush()
	# print 'hoi?'
	if alg_str=='astar':
		net = Net(board.gates[start], board.gates[end], i)
		alg = AStar(board, net)
		net = alg.createPath()
	elif alg_str=='dijkstra':
		net = Net(start, end, i)
		alg = Dijkstra(board, net)
		net = alg.createPath()
	elif alg_str=='simple':
		net = Net(start, end, i)
		alg = EasyPath(board)
		net = alg.createPath(net)
		board.removeNetPath(net)
		
	if not net.path:
		print '\nFailed planning a path for net', i, '!'
		return False
	else:
		board.nets[net.net_id]=net
		board.setNetPath(net)
		return True