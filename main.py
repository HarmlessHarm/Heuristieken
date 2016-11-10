from Objects import Board, Gate, Net
# from Visualizer import Visualizer
import json


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
	        # print('x: '+ str(gate['x']) + ', y: ' + str(gate['y']) + ', id: ' + str(gate['id']))
	        #b.setElementAt(gate['x'], gate['y'], 0, gate['id'])
	    print b.getLayer(0)
	    # v = Visualizer(b)
	    # v.start()

if __name__ == '__main__':
	createBoard()
