from modules import *
import argparse
import numpy as np
import matplotlib.pyplot as plt



if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-b', '--board', type=int, choices=[1,2], help="Specify which board to use (1 or 2), default is print 1")
	parser.add_argument('-n', '--netlist', type=int, choices=[1,2,3,4,5,6,7], help="Specify which netlist to use (1-7), default is netlist 1")
	parser.add_argument('-a', '--algorithm', type=str, choices=['astar', 'dijkstra', 'simple'], help="Specify which algorithm to use, default is astar")
	parser.add_argument('-v', '--visualization', action='store_true', help="3d visualization, default is off")
	args = parser.parse_args()

	netlists = readNetlists()
	if args.netlist != None:	
		n_id = args.netlist
		netlist = netlists[n_id-1]
	else:
		n_id = 1
		netlist = netlists[n_id-1]

	if args.board != None:
		b_id = args.board
		board = createBoard(b_id-1,len(netlist))
	else:
		b_id = 1
		board = createBoard(b_id-1,len(netlist))
		
	if args.algorithm != None:
		alg = args.algorithm
	else:
		alg = 'astar'
	netlist = [(24, 12), (18, 13), (2, 5), (0, 15), (1, 21), (22, 10), (11, 12), (15, 13), (15, 10), (22, 18), (3, 0), (13, 19), (22, 8), (15, 4), (16, 21), (8, 18), (12, 20), (5, 17), (10, 4), (14, 1), (12, 13), (8, 23), (4, 0), (4, 3), (10, 20), (11, 7), (10, 5), (18, 21), (9, 23), (19, 9), (11, 15), (17, 11), (19, 8), (14, 6), (23, 20), (14, 5), (1, 22), (6, 9), (13, 11), (14, 7)]

	print 'Running', alg, 'on board', b_id, 'with netlist', n_id
	runAlgorithm(alg, board, netlist)

	if args.visualization:
		v = Visualizer(board)
		v.start()


	


