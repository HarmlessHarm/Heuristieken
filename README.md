# Heuristieken

## About

This python application works with in a virtual environment so all packages can be easily maintained and a certain python version can be set.

## Installing pip, virtualenv and visualisation package

### Linux
`sudo apt-get install python-pip python-tk`   
`sudo pip install virtualenv`   

### macOS
`sudo easy_install pip`   
`brew install python --with-brewed-tk` *(This needs confirmation)*   
`sudo pip install virtualenv`

### Windows
Please no

## Installation and set up
*Clone git repository*   
`git clone https://github.com/HarmlessHarm/Heuristieken.git`   
`cd Heuristieken`   
*Initialise and activate virtual environment*   
`virtualenv env`   
`source env/bin/activate`   
*Install all python packages that are needed*   
`pip install -r requirements.txt`


## Way of work
Make sure you activate your virtualenv before you start working on the project.   
When you install a new package use the `pip freeze > requirements.txt` command to export the package requirements to git.

## Using the GUI
If you run `python gui.py` a GUI will pop up with options that relate to the arguments
that can be given to main.py   
*Note that netlists 3-6 aren't compatible with board 0*

**Good options to try:**   
`All default:` Shows basic functionality in a reasonable run speed   
`board 0, netlist 0, layers 3, rest default:` Shows that recursive algorithm can find a solution even with just 3 layers.   
`gen 100, pop 1000, rest default:` Shows that if you increase the generations and populations it will find better results.   
`board 1, netlist 5, layers 10:` Shows it can solve the most difficult board and netlist.   

**Found solutions:**   
Board 0, netlist 0: layers 3+   
Board 0, netlist 1: layers 4+   
Board 0, netlist 2: layers 6+   
Board 1, netlist 3: layers 5+ (6 faster)   
Board 1, netlist 4: layers 6+   
Board 1, netlist 5: layers 10   

## Running main
`python main.py` runs standard script, main has following options   
`--help` show help file   
`-b`, `--board` Specify which board to use (0 or 1), default is 0   
`-n`, `--netlist` Specify which netlist to use (0-6), default is netlist 0   
`-a`, `--algorithm` Specify which algorithm to use, default is astar   
`-v`, `--visualisation` 3D visualisation, default is off   
`-r`, `--recursion` Enable recursion on net order, default is off   
`-G`, `--genetic` Specify if genetic optimization is wanted   
`-g`, `--generations` Specify how many generations will be simulated   
`-p`, `--population` Specify how big the population should be   
`-R`, `--read` Specify if you want to check for an existing board   