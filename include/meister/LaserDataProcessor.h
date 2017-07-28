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
Contains the class definition for the Laser Data Node described in the updated requirments documnt
*/

#ifndef MEISTER_LASERDATAPROCESSOR_H
#define MEISTER_LASERDATAPROCESSOR_H

#include "NodeController.h"
#include "RosCommunications.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <tf/transform_listener.h>

/** Node Controller for processing Laser data from the catvehicle
 *
 */
class LaserDataProcessor : public NodeController {
private:

    ros::NodeHandle handle; //!< used for setting up ros communications and retreiving parameters
    tf::TransformListener listener; //!< listen for transforms needed to transform the LaserScan to point clouds

    int input_count; //!< count of messages received, used to limiting updates to the case where new messages have not yet been received

    RosCommunicator<sensor_msgs::LaserScan, sensor_msgs::PointCloud> talker; //!< handles ros communications for the node

    double range_max_; //!< disregard laser points further away than this


    /** fetches ROS parameters associated with this node
     *
     */
    void load_params();

    /** used to update the output message and sets output_needs_update_ to false if output_needs_update_ is true.
     *
     *  converts the laser_scan to a point cloud in the catvehicle/odom coordinate system for consumption by the obstacle detector
     */
     sensor_msgs::PointCloud create_point_cloud(const sensor_msgs::LaserScan &scan);

public:
    /** constructor
     *
     *
     */
    LaserDataProcessor();


    /** calls spinOnce, then exectues the filter logic and publishes the resultant point cloud
     *
     */
    void update() override;

    /** get the name of this node
     *
     * @return
     */
    std::string controller_name() override;
};


#endif //MEISTER_LASERDATAPROCESSOR_H
