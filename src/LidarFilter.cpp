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
Implementation of the Lidar Filter class

*/

#include "../include/meister/LidarFilter.h"
#include "MeisterCommon.h"

const double LidarFilter::DEFAULT_MIN_HEIGHT_ = 0.20;
const double LidarFilter::DEFAULT_MIN_DISTANCE_ = 10.0;
const double LidarFilter::DEFAULT_MAX_DISTANCE_ = 35.0;

using namespace meister::CommonStrings;
LidarFilter::LidarFilter() : handle_("~"), communicator_(CATVEHICLE_LIDAR_TOPIC,LIDAR_FILTER_OUTPUT_TOPIC, handle_){

    input_count_ = communicator_.input_count();
    load_params();

}

sensor_msgs::PointCloud LidarFilter::filter_points(const sensor_msgs::PointCloud &input){

    sensor_msgs::PointCloud output;
    output.header = input.header;
    output.channels = input.channels;

    double distance;
    double height_offset;
    bool passed_height;
    bool passed_distance;

    tf::StampedTransform transform;

    try{
        listener_.waitForTransform(CATVEHICLE_ROOT_COORDINATE_FRAME,input.header.frame_id, input.header.stamp, ros::Duration(0.1));
        listener_.lookupTransform(CATVEHICLE_ROOT_COORDINATE_FRAME, input.header.frame_id, input.header.stamp, transform);
        height_offset = transform.getOrigin().getZ();
    } catch (tf::TransformException &ex) {
        ROS_ERROR("first call: %s", ex.what());
        ros::Duration(1.0).sleep();
    }


    for (int i = 0; i < input.points.size(); i++){
        auto it = input.points[i];
        distance = point_distance(it);

        passed_height = true;
        passed_distance = true;


        if (it.z < min_height_ - height_offset){
            passed_height = false;
        }


        if (distance < min_distance_){
            passed_distance = false;
        }

        if (distance > max_distance_){
            passed_distance = false;
        }


        if (passed_distance && passed_height){
            output.points.push_back(it);
        }

    }

    return output;

}

double LidarFilter::point_distance(const geometry_msgs::Point32 &point){
    return sqrt(pow(point.x,2) + pow(point.y,2));
}

void LidarFilter::load_params(){
    if (!handle_.getParam("min_height", min_height_)){
        ROS_DEBUG("%s", handle_.getNamespace().c_str());
        min_height_ = DEFAULT_MIN_HEIGHT_;
    } else {
        ROS_DEBUG("min height: %f",min_height_);
    }
    handle_.param<double>("min_distance", min_distance_, DEFAULT_MIN_DISTANCE_);
    handle_.param<double>("max_distance", max_distance_, DEFAULT_MAX_DISTANCE_);
}

void LidarFilter::update(){
    ros::spinOnce();
    ROS_DEBUG("LidarFilter::update communicator_.input().points.size(): %d", static_cast<int>(communicator_.input().points.size()));
    if (input_count_ < communicator_.input_count()){
        input_count_ = communicator_.input_count();
        auto output_points = filter_points(communicator_.input());
        ROS_DEBUG("LidarFilter::update output_points.points.size(): %d", static_cast<int>(output_points.points.size()));
        communicator_.publish(filter_points(communicator_.input()));
    }
}

const double &LidarFilter::get_min_height() {
    return  min_height_;
}

const double &LidarFilter::get_min_distance() {
    return min_distance_;
}

const double &LidarFilter::get_max_distance() {
    return max_distance_;
}

std::string LidarFilter::controller_name(){
    return ros::this_node::getName();
}
