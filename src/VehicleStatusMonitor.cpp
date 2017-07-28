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

 Implementation of the VehicelStatusMonitor class, which contains the functionality of the VehicleStatusMonitorNode

*/

#include "VehicleStatusMonitor.h"
#include <math.h>
#include <iostream>

VehicleStatusMonitor::VehicleStatusMonitor(): handle("vehicle_status_monitor"),
                                              communicator("/catvehicle/vel", "/meister/vehicle_status", handle),
                                              status(VehicleStatus::UNKNOWN),
                                              in_motion_threshold_(0.75),
                                              stopped_threshold_(0.1),
                                              input_count_(0) {
    load_params();

}

void VehicleStatusMonitor::update() {
    ROS_DEBUG("VehicleStatusMonitor::update");
    ros::spinOnce();
    std_msgs::String message;

    if (communicator.input_count() > input_count_){
        input_count_ = communicator.input_count();
        // velocity gets set to near zero values when not commanded
        // this will round it to the nearest hundredth which should
        // make it zero when it's broadcasting the weird near zero values
        double velocity = communicator.input().linear.x;

        ROS_DEBUG("velocity message received: %f", velocity);
        if (status == VehicleStatus::UNKNOWN){

            status = VehicleStatus::STARTED;
            ROS_INFO("Vehicle Status: STARTED");
            message.data = "started";
            communicator.publish(message);

        }

        if (status != VehicleStatus::IN_MOTION && velocity > in_motion_threshold_){

            status = VehicleStatus::IN_MOTION;
            ROS_INFO("Vehicle Status: IN_MOTION");
            message.data = "in_motion";
            communicator.publish(message);

        } else if (status == VehicleStatus::IN_MOTION && velocity <= stopped_threshold_){

            status = VehicleStatus::STOPPED;
            ROS_INFO("Vehicle Status: STOPPED");
            message.data = "stopped";
            communicator.publish(message);

        }
    }
}


void VehicleStatusMonitor::load_params(){
    if (!handle.getParam("/vehicle_status_monitor/in_motion_threshold", in_motion_threshold_)){
        ROS_WARN("Could not load %s. a default of %f will be used",
                 std::string("/vehicle_status_monitor/in_motion_threshold").c_str(), in_motion_threshold_);
    }

    if (!handle.getParam("/vehicle_status_monitor/stopped_threshold", stopped_threshold_)){
        ROS_WARN("Could not load %s. a default of %f will be used",
                 std::string("/vehicle_status_monitor/stopped_threshold").c_str(), stopped_threshold_);
    }
}

const double &VehicleStatusMonitor::get_in_motion_threshold() {
    return in_motion_threshold_;
}

const double &VehicleStatusMonitor::get_stopped_threshold() {
    return stopped_threshold_;
}

std::string VehicleStatusMonitor::controller_name() {
    return ros::this_node::getName();
}