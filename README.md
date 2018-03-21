# Variable autonomy for the taurobot robot
There are a few dependencies that need to be installed before the packages can work/compiled.

# Installing ROS dependencies 
```sh
rosdep install gscam
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install ros-kinetic-frontier-exploration
sudo apt-get install ros-kinetic-hector-slam
```

# Installing fuzzylite 6 cpp library 
The original fuzzy controller was made using the version 4.0 of fuzzylite library. However this older version does not seem to work in ubuntu 16.04 and ROS kinetic. Thus fuzzylite 6.0 can be used. WARNING: the fuzzy controller compiled with version 6 of fuzzylite was not tested in practice.
1) Do a `cd` into the fuzzylite folder.

2)  run `./build.sh release`

3) Use checkinstall to create a debian file instead of installing the library old school. It makes life easier e.g. you can install or remove library via apt-get. 
```sh
sudo apt-get install checkinstall
cd release
sudo checkinstall --pkgname=fuzzylight6
```
4) The library is installed.

# Installing taurob new drivers
For the taurob to work we need to use the new [drivers](https://github.com/taurob/taurobtrackerapi/tree/uecu) and more specifically the branch "uecu" (IMPORTANT!: do not forget to `git checkout uecu`).  These drivers require the clocks of the control units inside the robot to be synchronized via NTP with the computer that is running the ROS stack. More info in the taurob repository.
# NTP synch for new drivers and ROS
For that we can use chrony:  `sudo apt-get install chrony`. This synch process is also an important step because the clocks of the ocu and the robot computer need to synchronised in order for ROS to work properly, not just the drivers!

 - To edit the chrony configuration file `sudo gedit /etc/chrony/chrony.conf` and then `invoke-rc.d chrony restart` to make your changes take effect.

* Add the following lines to ‘/etc/chrony.conf’ (the order of the lines does not matter)

Robot computer as server: 
```
bindaddress 10.0.0.100
allow 10.0.0.0/24
local stratum 10
initstepslew 10 ocu
manual
```

ocu computer as client:
```
server taurobot minpoll 0 maxpoll 5 maxdelay 0.3 iburst
bindaddress 10.0.0.100
allow 10.0.0.0/24
local stratum 10
initstepslew 20 taurobot
```

You may have a discrepancy in system times for various machines. You can check one machine against another using: `ntpdate -q other_computer_ip`

# NOT YET UPDATED Running simulated experiment

1) Run MORSE simulation via the morse_arena.py in experiments_launch->world

2) run on ROS the morse_arena.launch file in launch directory

# Running real world experiment

1) run on ROS the robot_nav.launch file on the robot. This will run all the neccesery nodes to control the robot via HI, teleop, automy.

2) run on ROS the operator.launch file on the OCU computer.. This will run all the nodes to control the robot from the OCU (joystick needed).

3) run on ROS the mi_control.launch file on the robot. This will enable the MI control to run on the robot.
