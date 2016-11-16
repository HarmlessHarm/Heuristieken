from Objects import Board, Gate, Net
from Algorithms import *
# from Visualizer import Visualizer
import numpy as np
import json

def readNetlists(i):

	netlist_file = 'netlist.txt'

	with open(netlist_file, 'r') as netlist_data:
	    nd = netlist_data.readlines()
	    net_dict = {}
	    for line in nd:
	        if line[0] == 'n':
	            line = line.split(' = ')
	            net_dict[line[0]] = line[1]
	        else:
	            continue
	    print(net_dict)
		
def createBoard(layers):

	file = 'prints.json'

	with open(file) as json_data:

	    d = json.load(json_data)
	    board = d['prints'][0]
	    width = board['width']
	    height = board['height']
	    b = Board(width, height, layers)
	    for gate in board['gates']:
	    	g = Gate(gate['id']-1, gate['x'], gate['y'])
	    	b.gates[g.gate_id] = (g.x, g.y, g.z)
	        b.setElementAt(g, g.x, g.y)

	    return b
	    # v = Visualizer(b)
	    # v.start()

if __name__ == '__main__':
	netList1 = [(23, 4), (5, 7), (1, 0), (15, 21), (3, 5), (7, 13), (3, 23), (23, 8), (22, 13), (15, 17), (20, 10), (15, 8), (13, 18), (19, 2), (22, 11), (10, 4), (11, 24), (3, 15), (2, 20), (3, 4), (20, 19), (16, 9), (19, 5), (3, 0), (15, 5), (6, 14), (7, 9), (9, 13), (22, 16), (10, 7)]
	netList2 = [(12, 20), (23, 20), (6, 9), (15, 10), (12, 13), (8, 18), (1, 22), (10, 20), (4, 3), (10, 5), (17, 11), (1, 21), (22, 8), (22, 10), (19, 8), (13, 19), (10, 4), (9, 23), (22, 18), (16, 21), (4, 0), (18, 21), (5, 17), (8, 23), (18, 13), (13, 11), (11, 7), (14, 7), (14, 6), (14, 1), (24, 12), (11, 15), (2, 5), (11, 12), (0, 15), (14, 5), (15, 4), (19, 9), (3, 0), (15, 13)]
	netList3 = [(0, 13), (0, 14), (0, 22), (8, 7), (2, 6), (3, 19), (3, 9), (4, 8), (4, 9), (5, 14), (6, 4), (4, 1), (7, 23), (10, 0), (10, 1), (8, 1), (7, 5), (12, 14), (13, 2), (8, 10), (11, 0), (11, 17), (11, 3), (8, 9), (12, 24), (13, 4), (13, 19), (15, 21), (10, 3), (18, 10), (24, 23), (16, 7), (17, 15), (17, 21), (17, 9), (18, 20), (18, 2), (12, 9), (1, 13), (19, 21), (20, 6), (1, 15), (2, 16), (20, 16), (22, 11), (22, 18), (2, 3), (5, 12), (24, 15), (24, 16)]
	board = createBoard(len(netList1))
	count = 0
	for i, (start,end) in enumerate(netList1):
		net = Net(start, end, i)

		easyPath = EasyPath(board)
		net = easyPath.createPath(net)

		dijkstra = Dijkstra()
		net = dijkstra.createPath(net)

		board.nets[i] = net
		print  i,'(',net.start_gate + 1,'->' ,net.end_gate + 1,') => ', net.path, '\n'
		if net.path == False:
			count += 1

	print "Invalid paths:",count
	print "Score for this board (number of paths, total length): ", board.getScore()



