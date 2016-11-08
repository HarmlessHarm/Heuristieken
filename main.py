import Board
import json


file = 'prints.json'

with open(file) as json_data:
	d = json.load(json_data)
	board = d['prints'][0]
	width = board['width']
	height = board['height']
	b = Board.Board(width, height)
	for gate in board['gates']:
		print('x: '+ str(gate['x']) + ', y: ' + str(gate['y']) + ', id: ' + str(gate['id']))
		#b.setElementAt(gate['x'], gate['y'], 0, gate['id'])
	b.prettyPrintLayer(0)
