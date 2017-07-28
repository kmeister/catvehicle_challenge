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
This file contains the class definition for a Node which
subscribes to /meister/filtered_lidar_points and publishes a copy transformed
into the /catvehicle/odom coordinate system to the
/meister/transformed_lidar_points topic.

 */

#ifndef MEISTER_STATEUPDATE_H_H
#define MEISTER_STATEUPDATE_H_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "NodeController.h"
#include "RosCommunications.h"

/** implements Node Controller which subscribes to /meister/filtered_lidar_points and publishes a copy transformed
 * into the /catvehicle/odom coordinate system to the /meister/transformed_lidar_points topic.
 *
 */
class StateUpdate : public NodeController{
private:

    ros::NodeHandle handle_; //!< node handle for this node
    RosCommunicator<sensor_msgs::PointCloud, sensor_msgs::PointCloud> communicator_; //!< handles communication with other ros nodes

    int input_count_; //!< number of messages received, used to prevent publishing new outputs for the same input


    tf::TransformListener listener_; //!< used to get transforms

public:

   /** this is the callback function for the subscriber it copies and transforms the inbound message to the
   *    /catvehicle/odom coordinate system before storing it in the outbound message buffer. sets the outbound_updated
   *    flag to true
   *
   * @param lidar_points point cloud to transofrm
   */
    sensor_msgs::PointCloud transform_points(const sensor_msgs::PointCloud& lidar_points);


    /** calls spin once and then if a new input point cloud has been received, transforms it and publishes the result
     *
     */
    void update() override;

    /** get this nodes name
     *
     * @return this node's name
     */
    std::string controller_name() override;

    /** Constructor
     *
     *
     */
    StateUpdate();
};


#endif //MEISTER_STATEUPDATE_H_H
