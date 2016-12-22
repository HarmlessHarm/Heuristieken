# -*- coding: utf-8 -*-
from modules import *
import matplotlib.pyplot as plt
import numpy as np
import copy
import time
import random

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

def randomNet(board):
    keys = board.gates.keys()
    g1 = random.choice(keys)
    keys.remove(g1)
    g2 = random.choice(keys)
    return g1, g2

def algorithmBenchmark(n):
    board = createBoard(1, 10)
    astarStats = {}
    astarStart = time.time()
    for i in range(n):
        g1, g2 = randomNet(board)
        net = Net(g1, g2, 0)
        astar = AStar(board, net, bias=False)

        pathStart = time.time()
        net.path = astar.createPath()
        pathEnd = time.time()
        pathTime = pathEnd - pathStart

        coord1 = board.gates[g1].getCoordinates()
        coord2 = board.gates[g2].getCoordinates()
        manHatScore = abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])

        if manHatScore in astarStats.keys():
            astarStats[manHatScore].append(pathTime)
        else:
            astarStats[manHatScore] = [pathTime]
        # astarStats.append((manHatScore,pathTime))
    
    astarEnd = time.time()
    print "Astar: ", astarEnd - astarStart

    dijkstraStats = {}
    dijkStart = time.time()
    for i in range(n):
        g1, g2 = randomNet(board)
        net = Net(g1, g2, 0)
        astar = Dijkstra(board, net)

        pathStart = time.time()
        net.path = astar.createPath()
        pathEnd = time.time()
        pathTime = pathEnd - pathStart


        coord1 = board.gates[g1].getCoordinates()
        coord2 = board.gates[g2].getCoordinates()
        manHatScore = abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])

        if manHatScore in dijkstraStats.keys():
            dijkstraStats[manHatScore].append(pathTime)
        else:
            dijkstraStats[manHatScore] = [pathTime]

    dijkEnd = time.time()
    print "Dijkstra: ", dijkEnd - dijkStart

    astarPlot = ([],[],[])
    for i,l in astarStats.iteritems():
        astarPlot[0].append(i)
        astarPlot[1].append(np.mean(l))
        astarPlot[2].append(np.std(l))
    
    dijkstraPlot = ([],[],[])
    for i,l in dijkstraStats.iteritems():
        dijkstraPlot[0].append(i)
        dijkstraPlot[1].append(np.mean(l))
        dijkstraPlot[2].append(np.std(l))

    plt.errorbar(*astarPlot, linestyle='None', marker='^', color='b')
    plt.errorbar(*dijkstraPlot, linestyle='None', marker='^', color='r')
    # print zip(*astarStats)
    # plt.scatter(*zip(*astarStats), color='b')
    # plt.scatter(*zip(*dijkstraStats), color='r')
    plt.show()


algorithmBenchmark(10000)