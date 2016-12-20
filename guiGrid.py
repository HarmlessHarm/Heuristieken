from Tkinter import *
import tkMessageBox
import os
# import main

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

TITLEFONT = ('Helvetica', '14')
FONT = ('Helvetica', '10')

root = Tk()
root.config(pady=15, padx=20)
root.minsize(width=750, height=300)

introLabel = Label(root, text='Welkom bij de super awesome netlist solver', font=TITLEFONT)
introLabel.pack()

container = Frame(root, pady=10)
container.pack()

# Board
boardFrame = Frame(container, pady=5, padx=10)
boardFrame.grid(row=0, column=0)

boardLabel = Label(boardFrame, text='Board', font=FONT, padx=5)
boardLabel.pack( side = LEFT)

board = StringVar()
board.set('0')
boardOptions = OptionMenu(boardFrame, board, '0', '1')
boardOptions.config(font=FONT)
boardOptions.pack()
boardOps = boardOptions.nametowidget(boardOptions.menuname) 
boardOps.configure(font=FONT)


# Netlist
netlistFrame = Frame(container, pady=5, padx=10)
netlistFrame.grid(row=1, column=0)

netlistLabel = Label(netlistFrame, text='Netlist', font=FONT, padx=5)
netlistLabel.pack(side=LEFT)

netlist = StringVar()
netlist.set('0')
netlistOptions = OptionMenu(netlistFrame, netlist, '0', '1', '2', '3', '4', '5')
netlistOptions.pack()
netlistOptions.config(font=FONT)
netOps = netlistOptions.nametowidget(netlistOptions.menuname) 
netOps.configure(font=FONT)



# Layers
layersFrame = Frame(container, pady=5, padx=10)
layersFrame.grid(row=2, column=0)

layersLabel = Label(layersFrame, text="Layers", font=FONT, padx=5)
layersLabel.pack(side=LEFT)
layers = StringVar()
layers.set('5')
layersField = Entry(layersFrame, textvariable=layers, width=3, font=FONT)
layersField.pack(side=RIGHT)

# Recursive
recursiveFrame = Frame(container, pady=5, padx=10)
recursiveFrame.grid(row=0, column=1)

recursive = IntVar()
recursive.set(1)
recursiveCheck = Checkbutton(recursiveFrame, text='Recursive',variable=recursive, font=FONT)
recursiveCheck.pack()

# Visual
visualFrame = Frame(container, pady=5, padx=10)
visualFrame.grid(row=1, column=1)

visual = IntVar()
visual.set(1)
visualCheck = Checkbutton(visualFrame, text='Visual', variable=visual, font=FONT)
visualCheck.pack(side=LEFT)

# Read File
readFrame = Frame(container, pady=5, padx=10)
readFrame.grid(row=2, column=1)

read = IntVar()
read.set(1)
readCheck = Checkbutton(readFrame, text='Read file', variable=read, font=FONT)
readCheck.pack(side=LEFT)

# Genetic
geneticFrame = Frame(container, pady=5, padx=10)
geneticFrame.grid(row=0, column=2)

genetic = IntVar()
genetic.set(1)
geneticCheck = Checkbutton(geneticFrame, text='Genetic', variable=genetic, font=FONT)
geneticCheck.pack()

# Generations
genFrame = Frame(container, pady=5, padx=10)
genFrame.grid(row=1, column=2)

genLabel = Label(genFrame, text="Gens", font=FONT, padx=5)
genLabel.pack(side=LEFT)
gen = StringVar()
gen.set('50')
genField = Entry(genFrame, textvariable=gen, width=4, font=FONT)
genField.pack()

# Population
popFrame = Frame(container, pady=5, padx=10)
popFrame.grid(row=2, column=2)

popLabel = Label(popFrame, text="Pop", font=FONT, padx=5)
popLabel.pack(side=LEFT)
pop = StringVar()
pop.set('100')
popField = Entry(popFrame, textvariable=pop, width=5, font=FONT)
popField.pack()

run = Button(root, text='RUN', command=runMain, font=FONT)
run.pack(side=BOTTOM)

root.mainloop()