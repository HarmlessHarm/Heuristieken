from Objects import *
from Algorithms import *
from Visualizer import *
# import pkgutil, inspect

__all__ = ["Board", "Net", "Gate", "EasyPath", "AStar", "Dijkstra", "runAlgorithm", "Visualizer"]

# for loader, name, is_pkg in pkgutil.walk_packages(__path__):
# 	module = loader.find_module(name).load_module(name)

# 	for name, value in inspect.getmembers(module):
# 		if name.startswith('__'):
# 			continue

# 		globals()[name] = value
# 		__all__.append(name)

# print __all__
