# beginner_tutorials

## Author

Sri Manika Makam

## Overview

This is a beginner tutorial on creating a ROS publisher and subscriber. ROS publisher sends the messages and ROS subscriber received the messages. 

## Assumptions

ROS Kinetic is installed in your computer and catkin workspace is set up.

## Standard install via command-line and set up
```
- git clone --recursive https://github.com/manikamakam/beginner_tutorials.git
- Suppose your catkin workspace is 'catkin_ws' which has build, src and devel folders.
- Move the cloned folder to catkin_ws/src.
- Open terminal and run the following commands:
  cd ~/catkin_ws
  source ./devel/setup.bash
  catkin_make

```
## Steps to run the publisher and subscriber
```
- Open catkin_ws in a terminal and source your workspace's setup.sh file by running the following commands:
  cd ~/catkin_ws
  source ./devel/setup.bash
- Run the command: roscore
- Open new terminal and run the following commands to run the publisher:
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials talker
- Open new terminal and run the following commands to run the subscriber:
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials listener

```
## License
```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
