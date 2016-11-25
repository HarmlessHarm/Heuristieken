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
		if checkNetlist('astar', s.board, s.netlist) is True:
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
	# board = createBoard(0,30)
	board = runAlgorithm(alg_str, 0, netlist)
	print board
	try:
		print board.getScore()
	except e:
		print "error", e
	v = Visualizer(board)
	v.start()


netlist = [(19, 2), (2, 20), (3, 15), (1, 0), (15, 21), (3, 23), (20, 10), (16, 9), (5, 7), (15, 8), (11, 24), (23, 8), (3, 0), (7, 13), (10, 4), (22, 16), (9, 13), (10, 7), (13, 18), (3, 4), (22, 13), (3, 5), (7, 9), (23, 4), (19, 5), (15, 5), (6, 14), (22, 11), (15, 17), (20, 19)]
notvalid = [(12, 20), (23, 20), (6, 9), (15, 10), (12, 13), (8, 18), (1, 22), (10, 20), (4, 3), (10, 5), (17, 11), (1, 21), (22, 8), (22, 10), (19, 8), (13, 19), (10, 4), (9, 23), (22, 18), (16, 21), (4, 0), (18, 21), (5, 17), (8, 23), (18, 13), (13, 11), (11, 7), (14, 7), (14, 6), (14, 1), (24, 12), (11, 15), (2, 5), (11, 12), (0, 15), (14, 5), (15, 4), (19, 9), (3, 0), (15, 13)]
lalal =[(12, 13), (14, 7), (13, 11), (10, 5), (14, 6), (19, 8), (14, 1), (22, 10), (0, 15), (10, 4), (15, 4), (5, 17), (18, 13), (15, 13), (2, 5), (14, 5), (11, 12), (12, 20), (23, 20), (6, 9), (15, 10), (8, 18), (1, 22), (10, 20), (4, 3), (17, 11), (1, 21), (22, 8), (13, 19), (9, 23), (22, 18), (16, 21), (4, 0), (18, 21), (8, 23), (11, 7), (24, 12), (11, 15), (19, 9), (3, 0)]

# randomSamples(500, 0, 1)
astar(lalal, 'astar')