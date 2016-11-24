import json, ast, os
import numpy as np
from Objects import *

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