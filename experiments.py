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
		b = createBoard(bi, 30)
		s = Sorter(n,b)
		s.random()
		if runAlgorithm('astar', s.board, s.netlist):
			validLists.append(s.netlist)
			solved, score = s.board.getScore()

			with open(filename, 'a') as wf:
				wf.write(str(score)+ " : " + str(s.netlist) + "\n")
			l.append(score)
	# a = np.histogram(l)
	# print a
	# print validLists
	# plt.hist(l)
	# plt.show()

def astar(netlist, alg_str):
	board = createBoard(0,30)
	if runAlgorithm(alg_str, board, netlist):
		print board.getScore()
		v = Visualizer(board)
		v.start()


netlist = [(19, 2), (2, 20), (3, 15), (1, 0), (15, 21), (3, 23), (20, 10), (16, 9), (5, 7), (15, 8), (11, 24), (23, 8), (3, 0), (7, 13), (10, 4), (22, 16), (9, 13), (10, 7), (13, 18), (3, 4), (22, 13), (3, 5), (7, 9), (23, 4), (19, 5), (15, 5), (6, 14), (22, 11), (15, 17), (20, 19)]
randomSamples(500, 0, 0)
astar(netlist, 'astar')