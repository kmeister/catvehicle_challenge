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

 Implements the Node Controller base class

*/

#include <ros/ros.h>
#include <MeisterCommon.h>
#include "NodeController.h"


using namespace meister::CommonValues;

NodeController::NodeController() : should_stop_(false), run_rate_(DEFAULT_RUN_RATE_){};

void NodeController::run(){
    ros::Rate rate(run_rate_);

    while(!ros::isShuttingDown() && !should_stop_){
        update();

        if (rate.cycleTime() > rate.expectedCycleTime()){
            ROS_WARN("%s run loop timing overrun: %f", controller_name().c_str(), ros::Time::now().toSec());
        }
        rate.sleep();
    }

    on_shutdown();
}

void NodeController::stop() {
    should_stop_ = true;
}

void NodeController::update() {
    ROS_DEBUG("NodeController::update()");
}

void NodeController::on_shutdown() {
    ROS_DEBUG("NodeController::on_shutdown()");
}

double NodeController::get_rate(){
    return run_rate_;
}