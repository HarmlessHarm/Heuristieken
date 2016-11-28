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


## Running main
`python main.py` runs standard script, main has following options   
`--help` show help file   
`-b`, `--board` Specify which board to use (1 or 2), default is 1   
`-n`, `--netlist` Specify which netlist to use (1-7), default is netlist 1   
`-a`, `--algorithm` Specify which algorithm to use, default is astar   
`-v`, `--visualisation` 3D visualisation, default is off   
`-r`, `--recursion` Enable recursion on net order, default is off   