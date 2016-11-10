from __future__ import print_function
from Board import Board
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


file = 'prints.json'

with open(file) as json_data:

    d = json.load(json_data)

board = d['prints'][0]
width = board['width']
height = board['height']
b = Board(width, height)
for gate in board['gates']:
    b.setElementAt(gate['x'], gate['y'], 0, gate['id'])
    # print('x: '+ str(gate['x']) + ', y: ' + str(gate['y']) + ', id: ' + str(gate['id']))
    #b.setElementAt(gate['x'], gate['y'], 0, gate['id'])
# v = Visualizer(b)
# v.start()

