from modules import *
import matplotlib.pyplot as plt


def randomSamples(iterations, bi, ni):
	netlists = readNetlists()
	n = netlists[ni]
	l = []
	validLists = []
	filename = 'validLists-b'+str(bi+1)+'-n'+str(ni+1)+'.txt'
	for i in range(iterations):
		print i,
		b = createBoard(bi, 10)
		s = Sorter(n,b)
		s.random()
		if checkNetlist('astar', s.board, s.netlist) is True:
			validLists.append(s.netlist)
			solved, score = s.board.getScore()

			with open(filename, 'a') as wf:
				wf.write(str(score)+ " : " + str(s.netlist) + "\n")
			l.append(score)

def astar(netlist, alg_str):
	board = runAlgorithm(alg_str, 0, netlist, 5, recursive=True)
	print board
	try:
		print board.getScore()
	except e:
		print "error", e
	# v = Visualizer(board)
	# v.start()

	return board


def genetic(board, netlist):
	gen = GeneticOpt(board, netlist, 1, 2)
	print gen.population
	gen.run()


netlist = [(15, 8), (3, 15), (15, 5), (20, 19), (23, 4), (5, 7), (1, 0), (15, 21), (3, 5), (7, 13), (3, 23), (23, 8), (22, 13), (15, 17), (20, 10), (13, 18), (19, 2), (22, 11), (10, 4), (11, 24), (2, 20), (3, 4), (16, 9), (19, 5), (3, 0), (6, 14), (7, 9), (9, 13), (22, 16), (10, 7)]
netlist2 = [(12, 20), (23, 20), (6, 9), (15, 10), (12, 13), (8, 18), (1, 22), (10, 20), (4, 3), (10, 5), (17, 11), (1, 21), (22, 8), (22, 10), (19, 8), (13, 19), (10, 4), (9, 23), (22, 18), (16, 21), (4, 0), (18, 21), (5, 17), (8, 23), (18, 13), (13, 11), (11, 7), (14, 7), (14, 6), (14, 1), (24, 12), (11, 15), (2, 5), (11, 12), (0, 15), (14, 5), (15, 4), (19, 9), (3, 0), (15, 13)]
lastNet2 = [(12, 13), (14, 7), (13, 11), (10, 5), (14, 6), (19, 8), (14, 1), (22, 10), (0, 15), (10, 4), (15, 4), (5, 17), (18, 13), (15, 13), (2, 5), (14, 5), (11, 12), (12, 20), (23, 20), (6, 9), (15, 10), (8, 18), (1, 22), (10, 20), (4, 3), (17, 11), (1, 21), (22, 8), (13, 19), (9, 23), (22, 18), (16, 21), (4, 0), (18, 21), (8, 23), (11, 7), (24, 12), (11, 15), (19, 9), (3, 0)]
netlist = readNetlists()[0]
# randomSamples(500, 0, 0)
# board = astar(netlist, 'astar')
# genetic(board, netlist)