from Tkinter import *
import tkMessageBox
import os
import main

def runMain():
	b = board.get()
	n = netlist.get()
	l = layers.get()
	g = gen.get()
	p = pop.get()
	command = 'python main.py -b ' + b + ' -n ' + n + ' -l ' + l
	if genetic.get():
		command += ' -G -g ' + g + ' -p ' + p
	if recursive.get():
		command += ' -r'
	if visual.get():
		command += ' -v'
	print 'running "'+command+'"'
	os.system(command)


root = Tk()
root.minsize(width=200, height=200)

introLabel = Label(root, text='Welkom bij de super awesome netlist solver')
introLabel.pack()

frameLeft = Frame(root)
frameLeft.pack(side=LEFT)

# Board
boardFrame = Frame(frameLeft)
boardFrame.pack()

boardLabel = Label(boardFrame, text='Board')
boardLabel.pack( side = LEFT)

board = StringVar()
board.set('0')
boardOptions = OptionMenu(boardFrame, board, '0', '1')
boardOptions.pack()

# Netlist
netlistFrame = Frame(frameLeft)
netlistFrame.pack()

netlistLabel = Label(netlistFrame, text='Netlist')
netlistLabel.pack(side=LEFT)

netlist = StringVar()
netlist.set('0')
netlistOptions = OptionMenu(netlistFrame, netlist, '0', '1', '2', '3', '4', '5')
netlistOptions.pack()

# Layers
layersFrame = Frame(frameLeft)
layersFrame.pack()

layersLabel = Label(layersFrame, text="Layers")
layersLabel.pack(side=LEFT)
layers = StringVar()
layers.set('5')
layersField = Entry(layersFrame, textvariable=layers, width=5)
layersField.pack()


frameRight = Frame(root)
frameRight.pack(side = RIGHT)
# Recursive
recursiveFrame = Frame(frameRight)
recursiveFrame.pack()

recursive = IntVar()
recursive.set(1)
recursiveCheck = Checkbutton(recursiveFrame, text='Recursive',variable=recursive)
recursiveCheck.pack()

# Visual
visualFrame = Frame(frameRight)
visualFrame.pack()

visual = IntVar()
visual.set(1)
visualCheck = Checkbutton(visualFrame, text='Visual', variable=visual)
visualCheck.pack()

# Genetic
geneticFrame = Frame(frameRight)
geneticFrame.pack()

genetic = IntVar()
genetic.set(1)
geneticCheck = Checkbutton(geneticFrame, text='Genetic', variable=genetic)
geneticCheck.pack()

# Generations
genFrame = Frame(frameRight)
genFrame.pack()

genLabel = Label(genFrame, text="Generations")
genLabel.pack(side=LEFT)
gen = StringVar()
gen.set('50')
genField = Entry(genFrame, textvariable=gen, width=5)
genField.pack()

# Population
popFrame = Frame(frameRight)
popFrame.pack()

popLabel = Label(popFrame, text="Population")
popLabel.pack(side=LEFT)
pop = StringVar()
pop.set('100')
popField = Entry(popFrame, textvariable=pop, width=5)
popField.pack()

run = Button(root, text='RUN', command=runMain)
run.pack(side=BOTTOM)

root.mainloop()