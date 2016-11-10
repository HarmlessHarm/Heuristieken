from Objects import Board, Gate, Net
# from Visualizer import Visualizer
import json

netlist_file = 'netlist.txt'

with open(netlist_file, 'r') as netlist_data:
    nd = netlist_data.readlines()
    net_dict = {}
    for line in nd:
        if line[0] == 'n':
            line = line.split(' = ')
            # print(line, end='')
            net_dict[line[0]] = line[1]
        else:
            continue
    print(net_dict)


def createBoard():

	file = 'prints.json'

	with open(file) as json_data:

	    d = json.load(json_data)
	    board = d['prints'][2]
	    width = board['width']
	    height = board['height']
	    b = Board(width, height)
	    for gate in board['gates']:
	    	g = Gate(gate['x'], gate['y'], gate['id'])
	        b.setElementAt(g.gate_id, g.x, g.y)

	    print b.getLayer(0)
	    return b
	    # v = Visualizer(b)
	    # v.start()

if __name__ == '__main__':
	board = createBoard()
	netList = [(1,3),(2,4)]
