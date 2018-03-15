# Variable autonomy for the taurobot robot
There are a few dependencies that need to be installed before the packages can work/compiled.

# Installing ROS dependencies 
```sh
rosdep install gscam
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install ros-kinetic-frontier-exploration
```

# Installing fuzzylite 6 cpp library 
The original fuzzy controller was made using the version 4.0 of fuzzylite library. However this older version does not seem to work in ubuntu 16.04 and ROS kinetic. Thus fuzzylite 6.0 can be used. WARNING: the fuzzy controller compiled with version 6 of fuzzylite was not tested in practice.
1) Do a `cd` into the fuzzylite folder.

2)  run `./build.sh release`

3) Use checkinstall to create a debian file instead of installing the library old school. It makes life easier e.g. you can install or remove library via apt-get. 
```sh
sudo apt-get install checkinstal
sudo checkinstall --pkgname=fuzzylight6
```
4) The library is installed.

# NOT YET UPDATED Running simulated experiment

1) Run MORSE simulation via the morse_arena.py in experiments_launch->world

2) run on ROS the morse_arena.launch file in launch directory

# NOT YET UPDATED Running real world experiment

1) run on ROS the robot_nav.launch file on the robot. This will run all the neccesery nodes to control the robot via HI, teleop, automy.

2) run on ROS the operator.launch file on the OCU computer.. This will run all the nodes to control the robot from the OCU (joystick needed).

3) run on ROS the mi_control.launch file on the robot. This will enable the MI control to run on the robot.
