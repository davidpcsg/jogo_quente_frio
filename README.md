# Hot and Cold

This repository is part of the **Hot and Cold Game**. 

The Hot and Cold is a game where the player commands a wheeled robot (Turtlebot3 Waffle) with the brain (EMOTIV Insight 5 channel mobile EEG) guiding it to a goal. The purpose of **jogo_quente_frio** is to read a REST/JSON API with the EMOTIV signals and post commands to ROS (Robot Operating System) interface.

### Hot and Cold Game Repositories:

- emotivinterface
- emotivwebserver
- jogo_quente_frio

### jogo_quente_frio Requirements:

- Ububtu 16.04
- libcurl
- jsoncpp

### jogo_quente_frio Installation:

###### Install Ububtu 16.04

###### Install ROS:

$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh

###### Install Dependent ROS Packages:

$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make

###### Install Turtlebot3 Simulations:

$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make

###### Install jogo_quente_frio module:

$ cd ~/catkin_ws/src/
$ git clone https://github.com/davidpcsg/jogo_quente_frio.git
$ cd ~/catkin_ws && catkin_make

###### Environment

$ nano ~/.bashrc

add lines:

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export TURTLEBOT3_MODEL=waffle
export EMOTIV_SERVER_URL=http://192.168.0.104:1234/emotiv

###### Running

roslaunch turtlebot3_gazebo turtlebot3_world.launch
rosrun jogo_quente_frio jogo_quente_frio
rosrun jogo_quente_frio emotiv_reader

###### References

https://www.youtube.com/watch?v=ehtUb55Rmmg&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=1  
http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation
http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup


### jogo_quente_frio ROS Modules:

###### jogo_quente_frio:

Listen to emotiv_reader (ROS module) messages and forward commands to turtlebot3 through ROS interface.

###### emotiv_reader:

Read emotivinterface JSON API and publish messages to jogo_quente_frio module.