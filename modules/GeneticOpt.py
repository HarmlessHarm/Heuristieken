from Objects import *
from AStar import *
from AStarAllPaths import *
import sys
import copy
import random
import datetime
import time

class GeneticOpt(object):

    """A genetic algorithm for optimizing a board and net configuration.

    Args:
        base_board (:obj: Board): The unoptimized board
        max_generations (int): The number of generations to simulate
        max_population (int): The size of the population
    """

    def __init__(self, alg_str, base_board, max_generations, max_population):
        super(GeneticOpt, self).__init__()
        self.alg_str = alg_str
        self.base_board = base_board
        self.max_generations = max_generations
        self.max_population = max_population
        self.base_score = self.base_board.getScore()[1]
        self.population = self.initPop()
        ts = time.time()
        self.resultFile = ('resources/'
                           +
                           datetime.datetime.fromtimestamp(
                               ts).strftime('%Y-%m-%d %H:%M:%S')
                           + 'g' +
                           str(self.max_generations)
                           + 'p' + str(self.max_population) + '.csv')
        with open(self.resultFile, "a") as file:
            file.write("iteration, max_score, min_score \n")

    def initPop(self):
        """Initialize the population

        Return:
            list: A list of <max_generation> deep-copied board objects
        """
        pop = []
        for i in range(self.max_population):
            pop.append((copy.deepcopy(self.base_board), self.base_score))
        return pop

    def run(self):
        """Run the algorithm

        Return:
            :obj:Board: The board with the lowest score from the last generation
        """
        for i in range(self.max_generations):
            print "In Generation", i
            newPop = self.iteration(self.population)
            sortedPop = self.sortPop(newPop)
            print self.population[0][1]
            self.writeIterationResults(i)
            killedPop = self.killPop(sortedPop)
            self.population = self.repopulate(killedPop)
        print "Improved from", self.base_score, 'to', self.population[0][1]
        return self.population[0][0]

    def iteration(self, population):
        """Runs the A* path finding algorithm for each member of the population

        Args:
            population (int): The number of individuals in each generation

        Return:
            list :A list with the new population as tuples of (board, score)
        """
        newPop = []
        for i, (board, score) in enumerate(population):
            if i % (len(population)/10) == 0:
                print '.',
                sys.stdout.flush()
            net = random.choice(board.nets)
            oldPath = copy.deepcopy(net.path)
            board.removeNetPath(net)
            if self.alg_str == 'astar':
                astar = AStarAllPaths(board, net)
                paths = astar.createPath()
                path = random.choice(paths)

                lengths = [len(p) for p in paths]
            else:
                bfs = BreadthFirst(net, board)
                paths = []
                paths = bfs.createPaths()
                if len(paths) > 0:
                    path = random.choice(paths)
                else:
                    print "No paths found"
                    path = []

            net.path = path
            if len(net.path) == 0:
                net.path = oldPath
                if not board.setNetPath(net):
                    print "OLD SHIT IS WRONG!!"
                newScore = board.getScore()[1]
                newPop.append((board, newScore))
            else:
                if not board.setNetPath(net):
                    print i, "SHIT IS WRONG!!!"
                newScore = board.getScore()[1]
                newPop.append((board, newScore))
        return newPop

    def sortPop(self, pop):
        return sorted(pop, key=lambda tup: tup[1])

    def killPop(self, pop):
        return pop[:len(pop)/2]

    def repopulate(self, pop):
        return copy.deepcopy(pop) + copy.deepcopy(pop)

    def writeIterationResults(self, iteration):
        with open(self.resultFile, "a") as file:
            max_score = self.population[0][1]
            min_score = self.population[len(self.population)-1][1]
            line = str(iteration) + ',' + str(max_score) + \
                ',' + str(min_score) + '\n'
            file.write(line)
