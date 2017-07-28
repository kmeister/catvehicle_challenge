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
    contains the definition for a ros node which monitors catvehicle velocity and publishes
    status messages when catvehicle/vel goes above or below parameterized thresholds
 */

#ifndef MEISTER_VEHICLESTATUSMONITOR_H
#define MEISTER_VEHICLESTATUSMONITOR_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "NodeController.h"
#include "RosCommunications.h"

/** enum describing the state of the catvehicle
 *
 */
enum class VehicleStatus {
    UNKNOWN = 0, //!< status of the vehicle before a /catvehicle/vel message is received
    STARTED, //!< catvehicle has not started moving yet
    IN_MOTION, //!< catvehicle has started moving
    STOPPED //!< catvehicle has come to a stop
};

/** monitors the state of the catvehicle and publishes messages indicating that the catvehicle
 * has started or stopped moving.
 *
 * messages are only published when the catvehicle transitions to a new state
 *
 */
class VehicleStatusMonitor : public NodeController {
private:
    ros::NodeHandle handle; //!< node handle for settu=ing up ros communications and loading params
    RosCommunicator<geometry_msgs::Twist, std_msgs::String> communicator; //!< used to get catvehicle velocity and publish vehicle status
    VehicleStatus status; //!< enumerated value representing vehicle status
    double in_motion_threshold_; //!< threshold above which the catvehicle will be considered to be in motion
    double stopped_threshold_; //!< threshold below which the catvehicle will be considered to be stopped
    int input_count_; //!< message counter used to determine if a new message has been received

public:
    /** constructor
     *
     */
    VehicleStatusMonitor();

    /** if a message has been recieved since it was last called, this method processes the message
     *  and publishes the appropriat resonse
     *
     */
    void update() override;

    /** loads ros params for in_motion_threshold_ and stopped_threshold_
     *
     */
    void load_params();

    /** accessor for in_motion_threshold_
     *
     * @return in_motion_threshold_
     */
    const double &get_in_motion_threshold();

    /** accessor for stopped_threshold_
     *
     * @return stopped_threshold_
     */
    const double &get_stopped_threshold();

    /** get this nodes name
     *
     * @return this node's name
     */
    std::string controller_name() override;

};


#endif //MEISTER_VEHICLESTATUSMONITOR_H
