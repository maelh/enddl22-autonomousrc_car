### What is this repository for? ###


### How do I get set up? ###
1. Install ROS indigo
2. Install V-REP
3. cloning this repo.
4. catkin_make

Then open a terminal and run V-REP by typing "./vrep.sh" at your V-REP installed path.

Open 4 terminals and run each command for each terminal.

~/indigo_catkin_ws/src/ackermann_robot/launch$ roslaunch ./teleop_key.launch
~/indigo_catkin_ws/src/ackermann_robot/launch$ roslaunch ./amcl.launch
~/indigo_catkin_ws/src/ackermann_robot/launch$ roslaunch ./mapping_20m.launch
~/indigo_catkin_ws/src/ackermann_robot$ rosrun rviz rviz -d ./rviz_config/amcl_rc_car.rviz

*If you want to see rostopic lists, then type "rostopic list -v"

"rosrun rqt_gui rqt_gui" will allow you to see images or plot by choosing visualization and add image_view and plot

*If you want to record data from V-REP, then type "rosbag record /vrep/image_raw" and multiple topics recording are also possible by "rosbag record -a"

*Extracting images from a bag file
1. create an empty folder.
2. move into the empty folder, and type "rosrun image_view image_saver image:=/vrep/image_raw"
3. open a new terminal and play bag file by "rosbag play ./your_bag_file_name.bag”