### What is this repository for? ###
This repository provides an Ackermann type of RC car simulation environment using V-REP and ROS. There are software packages integrated in order to make this happen.

* The V-REP RC car scene model is from [https://github.com/tkelestemur/ackermann_robot](https://github.com/tkelestemur/ackermann_robot), and only a map, and sensors configurations are slightly modified. Other than that, the V-REP scene model is identical.

* [https://github.com/bartville/vrep_plugin_imu](https://github.com/bartville/vrep_plugin_imu) provides to publish ROS [sensor_msgs/IMU](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) topic message.
 
* [https://github.com/whoenig/vrep_ros_helper](https://github.com/whoenig/vrep_ros_helper) is used to provide extra simulated clock source. This software package simply publishes */clock* topic based on the current machine clock.

* [https://github.com/ros-planning/navigation](https://github.com/ros-planning/navigation) is also used for mapping, localisation, and path planning. Frankly speaking, these software packages contain much information and may be a bit complicated those who isn't familiar with the basic understandings of some related theories. However there are awesome [tutorials](http://wiki.ros.org/navigation/Tutorials) provided by ROS and [other resources](https://www.packtpub.com/hardware-and-creative/mastering-ros-robotics-programming) may help you get through the challenges.

* [https://github.com/raulmur/ORB_SLAM](https://github.com/raulmur/ORB_SLAM) is the-state-of-the-art Monocular SLAM open source software package and it is not necessary to run the simulator, but it is a good example to demonstrate the images from V-REP can be utilised for other vision-based application.

![asdfasdf.png](https://bitbucket.org/repo/oa7zMk/images/590262749-asdfasdf.png)
![Screen Shot 2016-05-31 at 11.00.26 AM.png](https://bitbucket.org/repo/oa7zMk/images/1140182399-Screen%20Shot%202016-05-31%20at%2011.00.26%20AM.png)

## Here are the video demonstrations of these packages##

Autonomous navigation using a laser scanner
[https://youtu.be/ZiQwZdrpnlI](https://youtu.be/ZiQwZdrpnlI)

Up-to-scale poses estimation using ORB-SLAM
[https://youtu.be/xfFewdoVVbQ](https://youtu.be/xfFewdoVVbQ)




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