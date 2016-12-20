# -*- coding: utf-8 -*-
from modules import *
import matplotlib.pyplot as plt
import copy


def randomSamples(iterations, bi, ni):
    netlists = readNetlists()
    n = netlists[ni]
    l = []
    validLists = []
    filename = 'validLists-b'+str(bi+1)+'-n'+str(ni+1)+'.txt'
    for i in range(iterations):
        print i,
        b = createBoard(bi, 10)
        s = Sorter(n, b)
        s.random()
        if checkNetlist('astar', s.board, s.netlist) is True:
            validLists.append(s.netlist)
            solved, score = s.board.getScore()

            with open(filename, 'a') as wf:
                wf.write(str(score) + " : " + str(s.netlist) + "\n")
            l.append(score)


def astar(alg_str, boardN, netN):
    netlist = readNetlists()[netN]
    board = runAlgorithm(alg_str, boardN, netlist, 3, recursive=True)
    try:
        print board.getScore()
    except e:
        print "error", e
    # v = Visualizer(board)
    # v.start()

    return board


def genetic(board, gens, pop):
    gen = GeneticOpt('astar', board, gens, pop)
    # print gne.population
    genBoard = gen.run()
    dumpBoard(genBoard, 'astar', genetic=True, gen=gens, pop=pop)
    return genBoard


# board = findBoard('astar', 1, 1, 10)
# if not board:
board = astar('astar', 0, 0)
# genBoard = findBoard('astar' , 1, 1, 10, genetic=True, gen=100,pop=100)
# if not genBoard:
# nb = copy.deepcopy(board)
genBoard = genetic(board, 50, 1000)
# print genBoard

v = Visualizer(board)
v.start()

vg = Visualizer(genBoard)
vg.start()
