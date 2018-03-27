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
The original fuzzy controller was made using the version 4.0 of fuzzylite library. However this older version does not seem to work in ubuntu 16.04 and ROS kinetic. Thus fuzzylite 6.0 can be used. WARNING: the fuzzy controller to be compiled with version 6 of fuzzylite was not tested in practice. Some code changes were neccessery fot it to compile, hence the behivior might have changed (unlikely though!).
1) Do a `cd` into the fuzzylite folder.

2)  run `./build.sh release`

3) Use checkinstall to create a debian file instead of installing the library old school. It makes life easier e.g. you can install or remove library via apt-get. 

```sh
sudo apt-get install checkinstall
cd release
sudo checkinstall --pkgname=fuzzylight6
```

Alternatively `sudo make install`. In order to unistall if make install was used, `cat install_manifest.txt | xargs echo sudo rm | sh`.

4) The library is installed.

# Installing taurob new drivers
For the taurob to work we need to use the new [drivers](https://github.com/taurob/taurobtrackerapi/tree/uecu) and more specifically the branch "uecu" (IMPORTANT!: do not forget to `git checkout uecu`).  These drivers require the clocks of the control units inside the robot to be synchronized via NTP with the computer that is running the ROS stack. More info in the taurob repository.

# Network setup
A typical setup includes 2 computers. The on-board taurobot computer and the computer used by the operator (AKA operator control unit - OCU). The robot on-board computer has a static ip 10.0.0.3. The convension we will follow is to set a static ip 10.0.0.4 for the OCU via ubuntu network manager.

Once the static ips are in place we do `sudo gedit /etc/hosts` on both computers in order to add the host names. Add one of the following:

In computer used as OCU: 

127.0.1.1 	ocu

10.0.0.3 	taurobot

in robots computer:

127.0.1.1	taurobot

10.0.0.4        ocu

Normally we need the roscore to start on the taurobot computer. For this to happen `gedit ~/.bashrc` in both computers. Add the line `export ROS_MASTER_URI=http://taurobot:11311`.


# NTP synch for new drivers and ROS
Chrony can be used to synch the two computer and the sensors:  `sudo apt-get install chrony`. This synch process is also an important step because the clocks of the ocu and the robot computer need to synchronised in order for ROS to work properly, not just the drivers!

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

1) Run on the taurob computer `roslaunch taurob_mi_launch robot_nav.launch` or `roslaunch taurob_mi_launch robot_slam.launch`. This will run all the neccesery nodes to control the robot via HI, teleop, automy.

2) Run on the OCU `roslaunch taurob_mi_launch ocu.launch`. This will run all the nodes to control the robot from the OCU (joystick needed).

3) Run on the OCU `rosrun taurob_watchdog_client taurob_watchdog_client 10.0.0.3`. This will run the watchdog (for safety) client node.

4) run on ROS the mi_control.launch file on the robot. This will enable the MI control to run on the robot.
