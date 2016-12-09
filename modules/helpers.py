import json, ast, os
import numpy as np
from Objects import *
from Algorithms import *
import sys
# Try using cpickle lib for efficiency
try:
	import cpickle as pickle
except:
	import pickle

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

def dumpBoard(board, alg_str, fileName=None, genetic=False, gen=None, pop=None):
	netLen = len(board.nets)
	if board.y_dim == 13:
		boardN = 1
		if netLen == 30:
			netN = 1
		elif netLen == 40:
			netN = 2
		elif netLen == 50:
			netN = 3
		else:
			netN = 'x'
	elif board.y_dim == 17:
		boardN = 2
		if netLen == 50:
			netN = 4
		elif netLen == 60:
			netN == 5
		elif netLen == 70:
			netN = 6
		else:
			netN = 'x'
	else:
		boardN = 'x'
	d = './boards/'
	if fileName == None:
		fileName = 'board_' + str(alg_str) \
					+ '_b' + str(boardN) + '_n' + str(netN) \
					+ '_l' + str(board.z_dim) + '.pkl'
		if genetic:
			fileName = 'genetic_g'+str(gen)+'_p'+str(pop)+ '_' + fileName

	pickle.dump(board, open(d+fileName, 'wb'))
	print "Board stored in", d+fileName

def readBoard(fileName):
	d = './boards/'
	fileName = d + fileName
	board = pickle.load(open(fileName, 'rb'))
	return board

def findBoard(alg_str, boardN, netN, layerN, fileName=None, genetic=False, gen=None, pop=None):
	if fileName == None:
		fileName = 'board_' + str(alg_str) + '_b' + str(boardN) + '_n' + str(netN) + '_l' + str(layerN) + '.pkl'
		if genetic:
			fileName = 'genetic_g'+str(gen)+'_p'+str(pop)+ '_'+ fileName
	try:
		board = readBoard(fileName)
		return board
	except:
		return False

def runAlgorithm(alg_str, board_number, netlist, maxLayers, recursive=True):
	failedCount = 0
	board = createBoard(board_number, maxLayers)
	#print 'call runAlgorithm with', board
	i = checkNetlist(alg_str, board, netlist)
	if not i is True and recursive:
		newNetlist = [netlist[i]] + netlist[:i] + netlist[i+1:]
		board = runAlgorithm(alg_str, board_number, newNetlist, maxLayers)
	# dumpBoard(board, alg_str)
	return board

def checkNetlist(alg_str, board, netlist):
	failedCount = 0
	for i, (start, end) in enumerate(netlist):
		if i % round(len(netlist) / 20) == 0:
			print '.',
			sys.stdout.flush()
		if not checkPath(alg_str, board, start, end, i):
			# print netlist
			return i
	return True

def checkPath(alg_str, board, start, end, i):
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
		
	if not net.path:
		print '\nFailed planning a path for net', i, '!'
		return False
	else:
		board.nets[net.net_id]=net
		board.setNetPath(net)
		return True
