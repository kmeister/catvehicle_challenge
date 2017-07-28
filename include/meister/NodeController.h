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
        Defines a common base class for all nodes to use
*/
#ifndef MEISTER_ABSTRACTNODECONTROLLER_H
#define MEISTER_ABSTRACTNODECONTROLLER_H

#include <ros/ros.h>

/** Base class to be used to provide the main run loop for all of the nodes in this project. This should not be instantiated
 *  directly, but instead inherited from.
 *
 * a Node Controller is intended to manage the exectution of a ROS node within this project. All that is necessary
 * is to call ros::init, instantiate the node, and call run from the node's main function. The controller is intended to
 * handle everything else in a uniform way.
 *
 * TODO Add an  asynchronous run function that does not block
 * TODO add a wait for message function with a timeout
 */
class NodeController {
protected:
    int run_rate_; //!< max publication rate for the node
    bool should_stop_; //!< used to stop either run()

public:
    /** Constructor
     *
     */
    NodeController();

    /** The main "run-loop" for the function
     *
     *  run calls the node's update method and then sleeps for any time remaining in the cycle. Cycle time is determined by
     *  run_rate. this function blocks the calling thread until ros::isShuttingDown returns true, or stop() is called.
     *  After either stop or isShuttingDown, the on_shutdown function will be called.
     *
     */
    void run();



    /** Can be used to stop a running NodeController
     *
     */
    void stop();

    /** Abstract function to perform the periodic work of the NodeController this should not be called directly but
     * instead it will be invoked by the run() method.
     *
     */
    virtual void update();

    /** Abstract function to perform any final tasks after either stop() is called, or ros::isShuttingDown() returns true
     *
     */
    virtual void on_shutdown();

    /** get the name of this controller
     *
     * @return
     */
    virtual std::string controller_name() = 0;

    /** get execution freqeuncy
     *
     */
     double get_rate();
};



#endif //MEISTER_ABSTRACTNODECONTROLLER_H
