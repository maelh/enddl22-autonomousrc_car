### What is this repository for? ###
BlaBlaBla...

### How do I get set up? ###
* Install ROS indigo. Please follow instructions from [http://wiki.ros.org/indigo/Installation/Ubuntu](Link URL) if you haven't installed it.

* Install V-REP. Please download V-REP 3.3.1 from [http://www.coppeliarobotics.com/downloads.html](Link URL) to your favourite folder.

* cloning this repo by typing 
```
#!shell

git clone https://enddl22@bitbucket.org/enddl22/autonomousrc_car.git
```
* ROS package compilation by 
```
#!shell

catkin_make
```

### How can I run all packages? ###

* Running V-REP

open a terminal and run V-REP by typing 
```
#!shell

./vrep.sh
```
 at your V-REP installed path.

* Open 4 terminals and run following 4 commands for each terminal.


```
#!shell

~/indigo_catkin_ws/src/ackermann_robot/launch$ roslaunch ./teleop_key.launch
~/indigo_catkin_ws/src/ackermann_robot/launch$ roslaunch ./amcl.launch
~/indigo_catkin_ws/src/ackermann_robot/launch$ roslaunch ./mapping_20m.launch
~/indigo_catkin_ws/src/ackermann_robot$ rosrun rviz rviz -d ./rviz_config/amcl_rc_car.rvi
```

### How can I see rostopic and record data? ###
* If you want to see rostopic lists, then type

```
#!shell

rostopic list -v
```
* If you want to see images or plot by choosing visualization and add image_view and plot

```
#!shell

rosrun rqt_gui rqt_gui
```

* If you want to record data from V-REP, then type 
```
#!shell

rosbag record /vrep/image_raw
```
 and multiple topics recording are also possible by 
```
#!shell

rosbag record -a
```
## Extracting images from a bag file ##
* create an empty folder.
* move into the empty folder, and type 
```
#!shell

rosrun image_view image_saver image:=/vrep/image_raw
```

* open a new terminal and play bag file by 
```
#!shell

rosbag play ./your_bag_file_name.bag
```