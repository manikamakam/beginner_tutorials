/**
 * @file testTalker.cpp
 * @author Sri Manika Makam
 * @copyright BSD 3-Clause
 * @brief ROS test to test the talker node using g-test framework
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeString.h"
#include "std_msgs/String.h"

/**
 * @brief     Tests whether the changeString service exists or not
 * @param     testTalker        gtest framework
 * @param     testExistence     Name of the test
 */
TEST(testTalker, testExistence) {
  // Create the node handle
  ros::NodeHandle n;
  // Register a client to the changeString service
  auto client = n.serviceClient<beginner_tutorials::changeString>("changeString");
  // Tetss if the service exists or not
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief     Tests whether the changeString service can modify the output message
 * @param     testTalker            gtest framework
 * @param     testModifyMessage     Name of the test
 */
TEST(testTalker, testModifyMessage) {
  // Create the node handle
  ros::NodeHandle n;
  // Register a client to the changeString service
  auto client = n.serviceClient<beginner_tutorials::changeString>("changeString");
  // Initialize the service to srv object
  beginner_tutorials::changeString srv;
  // Modify the input string
  srv.request.inputString = "test";
  // Calling the client
  client.call(srv.request, srv.response);
  // Tests to see if the string is modified or not 
  EXPECT_STREQ("test", srv.response.newString.c_str());
}


