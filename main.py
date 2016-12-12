# -*- coding: utf-8 -*-
from modules import *
import argparse
import numpy as np
import matplotlib.pyplot as plt



if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-b', '--board', type=int, choices=[0,1], help="Specify which board to use (1 or 2), default is print 1")
	parser.add_argument('-n', '--netlist', type=int, choices=[0,1,2,3,4,5,6], help="Specify which netlist to use (1-7), default is netlist 1")
	parser.add_argument('-a', '--algorithm', type=str, choices=['astar', 'dijkstra', 'simple'], help="Specify which algorithm to use, default is astar")
	parser.add_argument('-v', '--visualization', action='store_true', help="3d visualization, default is off")
	parser.add_argument('-r', '--recursion', action='store_true', help="Enable recursion on net order, default is off")
	parser.add_argument('-dfs', '--depthfirstsearch', action='store_true', help="Enable depth first search on net order, default is off")
	parser.add_argument('-l', '--layers', type=int, help="Specify the maximum number of layers on the board, default is 10")
	parser.add_argument('-G', '--genetic', action='store_true', help="Specify if genetic optimization is wanted")
	parser.add_argument('-g', '--generations', type=int, help='specify how many generation')
	parser.add_argument('-p', '--population', type=int, help='specify how big the population should be')
	args = parser.parse_args()

	netlists = readNetlists()
	if args.netlist != None:	
		n_id = args.netlist
		netlist = netlists[n_id]
	else:
		n_id = 0
		netlist = netlists[n_id]

	if args.board != None:
		b_id = args.board
	else:
		b_id = 0

	if args.layers != None:
		l = args.layers
	else:
		l = 10
		
	if args.algorithm != None:
		alg = args.algorithm
	else:
		alg = 'astar'

	if args.genetic:
		if args.generations == None:
			gen = 50
		else:
			gen = args.generations
		if args.population == None:
			pop = 100
		else:
			pop = args.population
	# netlist = [(24, 12), (18, 13), (2, 5), (0, 15), (1, 21), (22, 10), (11, 12), (15, 13), (15, 10), (22, 18), (3, 0), (13, 19), (22, 8), (15, 4), (16, 21), (8, 18), (12, 20), (5, 17), (10, 4), (14, 1), (12, 13), (8, 23), (4, 0), (4, 3), (10, 20), (11, 7), (10, 5), (18, 21), (9, 23), (19, 9), (11, 15), (17, 11), (19, 8), (14, 6), (23, 20), (14, 5), (1, 22), (6, 9), (13, 11), (14, 7)]

	print 'Running', alg, 'on board', b_id, 'with netlist', n_id, 'and maximum number of layers:', l

	if args.depthfirstsearch:
		board = createBoard(b_id, l)
		dfs = DepthFirst(board, netlist)
		board, netlist = dfs.solve()
	else:
		board = runAlgorithm(alg, b_id, netlist, l, args.recursion)
		print '\nSolved', board.getScore()[0], 'nets with a total path length of:', board.getScore()[1]

	
	dumpBoard(board, alg)
	if args.genetic:
		gen = GeneticOpt('astar',board, gen, pop)
		genBoard = gen.run()

	if args.visualization:
		v = Visualizer(board)
		v.start()
		if args.genetic:
			vg = Visualizer(genBoard)
			vg.start()


	


