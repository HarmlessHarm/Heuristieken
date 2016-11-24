from modules import *

def randomSamples(iterations):
	netlists = readNetlists()
	n = netlists[0]
	l = []
	validLists = []
	for i in range(iterations):
		print i,
		b = createBoard(0, 30)
		s = Sorter(n,b)
		s.random()
		if runAlgorithm('astar', s.board, s.netlist, True):
			validLists.append(s.netlist)
			solved, score = s.board.getScore()
			l.append(score)
	# a = np.histogram(l)
	# print a
	print validLists
	plt.hist(l)
	plt.show()

randomSamples(5)