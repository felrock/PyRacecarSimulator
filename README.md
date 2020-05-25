# PyRacecarSimulator
Simulator for simulating a F1-tenth car. Itegrated with ROS and uses range_libc
for lidar simulations. Implemented to facilitate a Monte-Carlo Tree Search agent.

![](images/mcts.gif)

## Depends

### ROS
ROS melodic
Ackermann Messeges
LaserScan Messeges
Pose Messges

### Tensorflow

Tensorflow 1.13, with cudaNN 7.3

### Other libs

Clone range_libc and put the folder in this projects root.

## Installing

Put the project folder into a ros workspace and run,

```
catkin_make
source devel/setup.bash
```

## Usage

Tune the simulation and the agents using the param.yaml file. To change policy
for MCTS, edit **mcts.py** and swap out generateActionXX().


### Simulation only 


Build racecar
Build range_libc


To run the simulation as a ROS node run, 
```
roslaunch PyRacecarSimulator simulate.launch
```

### Simulation with MCTS


Build racecar
Build range_libc
Build followthegap


To run the simulation as a ROS node run, 
```
roslaunch PyRacecarSimulator simulate.launch
```


Then run the MCTS agent,
```
roslaunch PyRacecarSimulator mcts_driver.launch
```

# TODO
fix this readme
seperate mcts stuff and racecar simulator stuff
