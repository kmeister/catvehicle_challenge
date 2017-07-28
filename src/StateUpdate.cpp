/*
Author: Kurt Meister

Copyright (c) 2017, Kurt Meister
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

-------------------------------------------------------------------------------

Description:
This file contains the class definition and implementation for a Node which
subscribes to /meister/filtered_lidar_points and publishes a copy transformed
into the /catvehicle/odom coordinate system to the
/meister/transformed_lidar_points topic.

 */


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "StateUpdate.h"
#include "MeisterCommon.h"


using namespace meister::CommonStrings;

  void StateUpdate::update(){
    ros::spinOnce();

    if (communicator_.input_count() > input_count_){
      input_count_ = communicator_.input_count();
      try {
        sensor_msgs::PointCloud pc = transform_points(communicator_.input());
        communicator_.publish(transform_points(communicator_.input()));

      }
      catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0/get_rate()).sleep();
      }
    }

  }

  sensor_msgs::PointCloud StateUpdate::transform_points(const sensor_msgs::PointCloud& lidar_points){
    sensor_msgs::PointCloud outbound;
    tf::StampedTransform transform;

    std::string target_frame = "/catvehicle/odom";

    // access transforms for debug purposes

      listener_.transformPointCloud(target_frame, lidar_points, outbound );



    return outbound;
  }


  StateUpdate::StateUpdate(): handle_("state_update_node"),
                              communicator_(LIDAR_FILTER_OUTPUT_TOPIC, STATE_UPDATE_OUTPUT_TOPIC, handle_),
                              input_count_(communicator_.input_count()) {

  }

  std::string StateUpdate::controller_name() {
    return ros::this_node::getName();
  }
