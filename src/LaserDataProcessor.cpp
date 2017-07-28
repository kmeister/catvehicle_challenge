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

 Implementation of the LaserDataProcessor class which contains the functionality of the Laser Data Node

*/

#include "LaserDataProcessor.h"
#include "laser_geometry/laser_geometry.h"

void LaserDataProcessor::update() {
    ROS_DEBUG("LaserDataProcessor::update()");

    ros::spinOnce();

    if (talker.input_count() > input_count) {
        input_count = talker.input_count();
        talker.publish(create_point_cloud(talker.input()));
    }

}

LaserDataProcessor::LaserDataProcessor(): handle() ,
                                          talker("/catvehicle/front_laser_points","/meister/laser_points", handle),
                                          listener(handle){
    load_params();
    input_count = talker.input_count();
}

sensor_msgs::PointCloud LaserDataProcessor::create_point_cloud(const sensor_msgs::LaserScan &scan) {
    sensor_msgs::PointCloud final;
    laser_geometry::LaserProjection projector;

    if (!listener.waitForTransform("/catvehicle/odom", scan.header.frame_id,
                                   scan.header.stamp + ros::Duration().fromSec(scan.ranges.size()*scan.time_increment),
                                   ros::Duration(run_rate_))){
        ROS_WARN("Did not receive transform in time.");
        return final;
    }

    projector.transformLaserScanToPointCloud("/catvehicle/odom", scan, final, listener, range_max_,
                                             laser_geometry::channel_option::Distance );

    return final;
}

void LaserDataProcessor::load_params() {
    if (!handle.getParam("/laser_data/max_range", range_max_)){
        ROS_WARN("/laser_data/max_range not set using default of 45.0");
        range_max_ = 45.0;
    }
}

std::string LaserDataProcessor::controller_name() {
    return ros::this_node::getName();
}

