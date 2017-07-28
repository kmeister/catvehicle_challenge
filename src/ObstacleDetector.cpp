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
Contains the implementation of the ObstacleDetector class
 */

#include "ObstacleDetector.h"
#include "WorldBuilder.h"
#include <rosbag/bag.h>

using namespace std;

ObstacleDetector::ObstacleDetector() : grid_(0.5) , handle("obstacle_detector"), lidar_listener("/meister/transformed_lidar_points", handle),
                                       laser_listener("/meister/laser_points", handle),
                                       status_listener("/meister/vehicle_status", handle),
                                       write_world_file_(true){
    status_count = status_listener.input_count();
    lidar_count = lidar_listener.input_count();
    laser_count = laser_listener.input_count();

    load_params();
    detections_publisher = handle.advertise<geometry_msgs::PolygonStamped>("/detections", 1000);

}

ObstacleDetector::~ObstacleDetector() {
    if (world_creation_thread_){
        world_creation_thread_->join();
    }
}

vector<geometry_msgs::PolygonStamped> ObstacleDetector::detect_obstacles(const vector<geometry_msgs::Point32> &points) {
    vector<geometry_msgs::PolygonStamped> detections;
    geometry_msgs::PolygonStamped poly;

    if (!points.empty()){
        grid_.setPoints(points);

        auto labels = grid_.label_names();
        for (auto label: labels) {
            poly.polygon = grid_.get_axis_aligned_bounding_box(label);

            detections.push_back(poly);
        }
    }

    return detections;
}

void ObstacleDetector::update() {
    ros::spinOnce();
    all_points_mutex_.lock(); // allows world creation to happen in a thread safe way
    //publish detections if lidar points have been received
    if (lidar_listener.input_count() > lidar_count) {
        lidar_count = lidar_listener.input_count();

        auto point_cloud = lidar_listener.input();
        auto detections = detect_obstacles(point_cloud.points);

        for (auto it : detections){
            it.header = point_cloud.header;
            detections_publisher.publish(it);
        }

        all_points_.insert(all_points_.end(), lidar_listener.input().points.begin(), lidar_listener.input().points.end());
    }

    //publish detections if laser points have been recieved
    if (laser_listener.input_count() > laser_count){
        laser_count = laser_listener.input_count();

        auto point_cloud = laser_listener.input();
        auto detections = detect_obstacles(point_cloud.points);

        for (auto it : detections){
            it.header = point_cloud.header;
            detections_publisher.publish(it);
        }

        all_points_.insert(all_points_.end(), laser_listener.input().points.begin(), laser_listener.input().points.end());
    }
    all_points_mutex_.unlock();

    // detect vehicle status changed
    if (status_listener.input_count() != status_count)
    {
        status_count = status_listener.input_count();
        //ROS_WARN("write time: %f", ros::Time::now().toSec());

        if (status_listener.input().data == "stopped"){
            create_world_async();
        }

    }


}

void ObstacleDetector::create_world() {
    if (write_world_file_) {

        DetectionGrid grid(0.5);

        all_points_mutex_.lock(); // protect against call from separate thread
        grid.setPoints(all_points_);
        all_points_mutex_.unlock();

        geometry_msgs::PolygonStamped poly;

        WorldBuilder builder;
        builder.load_template(world_template_filename_);
        auto labels = grid.label_names();

        for (auto label : labels) {
            //cout << "Writing Polygon" << endl;
            poly.polygon = grid.get_axis_aligned_bounding_box(label);
            builder.insert_aabb(poly);

        }

        builder.write_world(world_output_filename_);
    }

    ROS_INFO("World File: %s written", world_output_filename_.c_str());

}

void ObstacleDetector::create_world_async() {
    // in case stopped is issued twice due to motion errors from gazebo
    if (world_creation_thread_){
        world_creation_thread_->join();
    }

    world_creation_thread_.reset(new boost::thread(&ObstacleDetector::create_world, this));
}

void ObstacleDetector::load_params() {

    if (!handle.getParam(handle.getNamespace() + "/world_template_filename", world_template_filename_)){
        ROS_WARN("World File Template Not Set, worldfile will not be created");
        write_world_file_ = false;
    }

    if (!handle.getParam(handle.getNamespace() + "/world_output_filename", world_output_filename_)){
        ROS_WARN("World Output File Not Set, worldfile will not be created");
        write_world_file_ = false;
    }

}

std::string ObstacleDetector::controller_name(){
    return ros::this_node::getName();
}