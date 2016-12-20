# from __future__ import
from Tkinter import *
import tkMessageBox
import os


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
    if read.get():
        command += ' -R'
    print 'running "'+command+'"'
    os.system(command)


def gridFrame(row, column):
    frame = Frame(container, pady=5, padx=10)
    frame.grid(row=row, column=column)
    return frame


def label(parent, text):
    Label(parent, text=text, font=FONT, padx=5).pack(side=LEFT)


def optionMenu(parent, text, values):
    label(parent, text)
    var = StringVar()
    var.set(values[0])
    optMenu = OptionMenu(parent, var, *values)
    optMenu.config(font=FONT)
    optMenu.pack()
    opts = optMenu.nametowidget(optMenu.menuname)
    opts.configure(font=FONT)
    return var


def entry(parent, text, default, width):
    label(parent, text)
    var = StringVar()
    var.set(default)
    field = Entry(parent, textvariable=var, width=width, font=FONT)
    field.pack(side=RIGHT)
    return var


def checkbox(parent, text, default=1):
    var = IntVar()
    var.set(default)
    field = Checkbutton(parent, text=text, variable=var, font=FONT)
    field.pack()
    return var


TITLEFONT = ('Helvetica', '14')
FONT = ('Helvetica', '10')

root = Tk()
root.config(pady=15, padx=20)
root.minsize(width=750, height=300)

introLabel = Label(root, text='Chips & Circuits by Team KI', font=TITLEFONT)
introLabel.pack()

container = Frame(root, pady=10)
container.pack()

# Board
boardFrame = gridFrame(0, 0)
board = optionMenu(boardFrame, 'Board', ['0', '1'])

# Netlist
netlistFrame = gridFrame(1, 0)
netlist = optionMenu(netlistFrame, 'Netlist', ['0', '1', '2', '3', '4', '5'])

# Layers
layersFrame = gridFrame(2, 0)
layers = entry(layersFrame, 'Layers', '5', 3)

# Recursive
recursiveFrame = gridFrame(0, 1)
recursive = checkbox(recursiveFrame, 'Recursive')

# Visual
visualFrame = gridFrame(1, 1)
visual = checkbox(visualFrame, 'Visual')

# Read File
readFrame = gridFrame(2, 1)
read = checkbox(readFrame, 'Read file', 0)

# Genetic
geneticFrame = gridFrame(0, 2)
genetic = checkbox(geneticFrame, 'Genetic')

# Generations
genFrame = gridFrame(1, 2)
gen = entry(genFrame, 'Gens', '50', 4)

# Population
popFrame = gridFrame(2, 2)
pop = entry(popFrame, 'Pop', '100', 5)

run = Button(root, text='RUN', command=runMain, font=FONT)
run.pack(side=BOTTOM)

root.mainloop()
