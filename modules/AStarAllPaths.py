from Objects import *
import numpy as np
import sys


class AStarAllPaths(object):

    """
    Initialize the A* algorithm with a board and net for which to plan a path, 
    optional is a bias parameter (either 'vertical' or 'lateral')

    Args:
        board (:obj: Board): The board on which to plan a path
        net (:obj: Net): The net for which to plan a path
        bias (:obj:'str', optional): A flag indicating which distance bias to use.
    """

    def __init__(self, board, net):
        super(AStarAllPaths, self).__init__()
        self.board = board
        self.net = net

    def createPath(self):
        """Run the algorithm to create a path

        Returns:
            list: empty if no path exists, ordered set of (x,y,z)-coordinates if there is.
        """
        (x, y, z) = self.board.getDimensions()
        start = self.board.gates[self.net.start_gate].getCoordinates()
        (x_start, y_start, z_start) = start
        goal = self.board.gates[self.net.end_gate].getCoordinates()
        (x_end, y_end, z_end) = goal
        # For each node, whether it has been evaluated
        closedSet = np.zeros((x, y, z), dtype=bool)

        # Set of discovered nodes that still need evaluation, where a node is a
        # tuple (x,y,z)
        openSet = [start]

        # Dictionary that for each node (a tuple x,y,z) archives which node it
        # can most easily be reached from
        cameFrom = {}

        # Array that contains g_scores for all nodes, the cost of getting from start to that node
        # default is -1, as placeholder for proper default infinity
        gScore = np.full((x, y, z), -1, dtype='int64')
        # Cost of going from start to start is 0
        gScore[x_start][y_start][z_start] = 0

        # array that contains f_scores for all nodes, the distance for getting to the goal node from start via that node
        # Default is -1, as placeholder for infinity
        fScore = np.full((x, y, z), -1, dtype='int64')
        fScore[x_start][y_start][
            z_start] = self.manhattanCostEstimate(start, goal)

        bestLen = sys.maxint
        goalFound = False
        while openSet != []:
            # Set currentNode to be the node in openset with the lowest fscore
            # (above -1)
            (cx, cy, cz) = openSet[0]
            for (x, y, z) in openSet:
                if 0 <= fScore[x][y][z] < fScore[cx][cy][cz]:
                    (cx, cy, cz) = (x, y, z)

            # if currentNode is adjacent to goal node, return the path to
            # currentNode
            if goal in self.board.getAllNeighbours(cx, cy, cz):
                if goal in cameFrom.keys():
                    cameFrom[goal].append((cx, cy, cz))
                else:
                    cameFrom[goal] = [(cx, cy, cz)]
                goalFound = True
                return self.reconstructPath(cameFrom, goal)

            openSet.remove((cx, cy, cz))

            for (nx, ny, nz) in self.board.getOpenNeighbours(cx, cy, cz):
                if closedSet[nx][ny][nz]:
                    continue  # neighbour is already evaluated

                tentative_gscore = gScore[cx][cy][
                    cz] + self.distance((nx, ny, nz))
                if not (nx, ny, nz) in openSet and not goalFound:
                    openSet.append((nx, ny, nz))
                elif tentative_gscore > gScore[nx][ny][nz] >= 0:
                    continue

                if (nx, ny, nz) in cameFrom.keys():
                    cameFrom[(nx, ny, nz)].append((cx, cy, cz))
                else:
                    cameFrom[(nx, ny, nz)] = [(cx, cy, cz)]
                gScore[nx][ny][nz] = tentative_gscore
                fScore[nx][ny][nz] = gScore[nx][ny][nz] + \
                    self.manhattanCostEstimate((nx, ny, nz), goal)

            closedSet[cx][cy][cz] = 1
        return self.reconstructPath(cameFrom, goal)

    def distance(self, node):
        """
        Distance to node, based on its neighbours and the layer it is in, 
        the bias in distance added is set when the objact is instantiated

        Args:
            node (tuple): a coordinate

        Returns:
            int: Distance to the node
        """
        (x, y, z) = node
        distance = 1
        # distance += z

        return distance

    # Very optimistic heuristic, it returns the manhattan distance between the
    # 2 nodes
    def manhattanCostEstimate(self, node1, node2):
        (x, y, z) = node1
        (x2, y2, z2) = node2
        return abs(x2-x)+abs(y2-y)+abs(z2-z)

    # Helper function to reconstruct the path
    def reconstructPath(self, cameFrom, currentNode):
        # pprint.pprint( cameFrom)

        paths = []
        allPaths = self.returnMultiplePaths(cameFrom, [currentNode], paths)
        return allPaths
        # path = [currentNode]
        # while currentNode in cameFrom.keys():
        #   currentNode = cameFrom[currentNode]
        #   path.append(currentNode)
        # return list(reversed(path))

    def returnMultiplePaths(self, cameFrom, path, paths):
        lastNode = path[-1]
        if len(paths) == 100:
            return paths
        if lastNode in cameFrom.keys():
            for nextNode in cameFrom[lastNode]:
                new_path = path + [nextNode]
                paths = self.returnMultiplePaths(cameFrom, new_path, paths)
        else:
            path = list(reversed(path))
            paths += [path]
        return paths

