# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Author

Sri Manika Makam

## Overview

This repository contains the implementation of basic ROS C++ Publisher and Subscriber.
ROS publisher sends the messages and ROS subscriber receives the messages.

Talker (src/talker.cpp): Publisher
Listener (src/listener.cpp): Subscriber

nodes.launch is the launch file which can be used to launch both talker and listener nodes.

changeString.srv is a service used to change the output string upon request by the user.

## Dependencies

ROS Kinetic should be installed on your computer (preferably Ubuntu 16.04).
Catkin workspace must be set up.

## Set up and build

- Suppose your catkin workspace is 'catkin_ws' which has build, src and devel folders.
- Open new terminal and run the command 
```
git clone --recursive https://github.com/manikamakam/beginner_tutorials.git

```
- Move the cloned folder to catkin_ws/src.
- Open terminal and run the following commands:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  catkin_make

```
## Steps to run the publisher and subscriber using rosrun

- Open catkin_ws in a terminal and source your workspace's setup.sh file by running the following commands:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  
```
- Run the following command to start the master
```
  roscore
  
```
- Open new terminal and run the following commands to run the publisher:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials talker

```
- Open new terminal and run the following commands to run the subscriber:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials listener

```
## Steps to run the publisher and subscriber at once using launch file

Make sure thte master is running and type the following command in a new terminal:
```
roslaunch beginner_tutorials nodes.launch

```
User can change the frequency at which the loop operates by the following command:
```
roslaunch beginner_tutorials nodes.launch frequency:=<desired_frequency>

```
Replace <desired_frequency> with the required number 

## Service to change the output string

User can change the output string message by running the following command in a new terminal:
```
rosservice call /changeString "text"

```
## Logging

To see the message log in real time, we use rqt_console GUI.
Type the following command in a new terminal:
```
rqt_console
```
