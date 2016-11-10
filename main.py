from Board import Board
from Visualizer import Visualizer
import json


file = 'prints.json'

with open(file) as json_data:
	d = json.load(json_data)
	board = d['prints'][0]
	width = board['width']
	height = board['height']
	b = Board(width, height)
	for gate in board['gates']:
		b.setElementAt(gate['x'], gate['y'], 0, gate['id'])
	v = Visualizer(b)
	v.start()
