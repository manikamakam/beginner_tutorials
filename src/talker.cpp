/**
 * @file talker.cpp
 * @author Sri Manika Makam
 * @copyright BSD 3-Clause
 * @brief Implementing the ROS publisher
 */

/**
 * The 3-Clause BSD License
 *
 * Copyright (c) 2019, Sri Manika Makam
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 */

#include <tf/transform_broadcaster.h>
#include "talker/talker.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"

/**
 * Providing a default string message which can be later modified by the user
 */
DefaultString def;

/**
 * @brief  Callback function for changeString Service
 * @param  req   Request data sent to service
 * @param  res   Response by the service to the client
 * @return bool
 */
bool newMessage(beginner_tutorials::changeString::Request &req,
                   beginner_tutorials::changeString::Response &res) {
  def.defaultString = req.inputString;
  ROS_WARN_STREAM("The user changed the string to");
  res.newString = req.inputString;
  return true;
}

/**
 * @brief  Callback function for tf broadcaster
 * @param  none
 * @return none
 */
void tfPoseCall() {
  // Transform broadcaster object
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // Set translation value
  transform.setOrigin(tf::Vector3(cos(ros::Time::now().toSec()),
                         sin(ros::Time::now().toSec()), 0.0));
  tf::Quaternion q;
  q.setRPY(1, 1, 0);

  // Set rotation value
  transform.setRotation(q);

  // Broadcast the transform
  br.sendTransform(tf::StampedTransform(transform,
                         ros::Time::now(), "world", "talk"));
}

/**
 * @brief main function
 * @param integer (argc) and character (argv) 
 * @return 0
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  // Set the default frequency to 20Hz
  int freq = 20;
  if (argc > 1) {
    // Convert the string to integer
    freq = atoi(argv[1]); }

  if (freq <= 1000 && freq > 0) {
    ROS_DEBUG_STREAM("Frequency rate is: "<< freq);
  } else if (freq > 1000) {
    ROS_ERROR_STREAM("Frequency rate is too large");
    ROS_WARN_STREAM("Setting the frequency to default value of 20Hz");
    // Set the frequency to default value of 20Hz
    freq = 20;
  } else if (freq < 0) {
    ROS_ERROR_STREAM("Frequency rate cannot be negative");
    ROS_WARN_STREAM("Setting the frequency to default value of 20Hz");
    // Set the frequency to default value of 20Hz
    freq = 20;
  } else if (freq == 0) {
    ROS_FATAL_STREAM("Frequency rate cannot be 0Hz");
    ROS_WARN_STREAM("Setting the frequency to default value of 20Hz");
    // Set the frequency to default value of 20Hz
    freq = 20; }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  auto server = n.advertiseService("changeString", newMessage);
  ros::Rate loop_rate(freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << def.defaultString << count;
    msg.data = ss.str();
    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // Calling the tfPoseCall function
    tfPoseCall();

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
