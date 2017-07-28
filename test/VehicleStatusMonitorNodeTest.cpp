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

Description: Contains Unit Test for the VehicleStatusMonitor

*/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "VehicleStatusMonitor.h"
#include <boost/thread/thread.hpp>

class VehicleStatusMonitorNodeTest : public ::testing::Test {
public:
    ros::NodeHandle handle;
    ros::Publisher pub;
    ros::Subscriber sub;

    std_msgs::String received;

    int r_count;

    boost::thread *thread;
    VehicleStatusMonitor *vehicle_status_monitor;

    void callback(const std_msgs::String &message){
        r_count++;
        received = message;
    }

    void wait_for_message(){
        int last = r_count;

        while(r_count == last){
            ros::spinOnce();
        }
    }

    bool wait_for_messages(double timeout, int count){
        int last = r_count;
        int spin_count = 0;

        ros::Rate r(100);

        while(r_count < last + count && spin_count / 100 < timeout){
            ros::spinOnce();
            spin_count++;
            r.sleep();
        }

        return last - r_count == count;
    }

    bool wait_for_message(double timeout){
        int last = r_count;
        int spin_count = 0;

        ros::Rate r(100);

        while(r_count == last && spin_count / 100 < timeout){
            ros::spinOnce();
            spin_count++;
            r.sleep();
        }

        return r_count != last;
    }

    VehicleStatusMonitorNodeTest() : handle("~") {
        pub = handle.advertise<geometry_msgs::Twist>("/catvehicle/vel",100);
        sub = handle.subscribe("/meister/vehicle_status",1000,&VehicleStatusMonitorNodeTest::callback, this);

        vehicle_status_monitor = new VehicleStatusMonitor();
        thread = new boost::thread(boost::bind(&VehicleStatusMonitor::run, vehicle_status_monitor));
    }

    ~VehicleStatusMonitorNodeTest(){
        vehicle_status_monitor->stop();
        thread->join();

        delete thread;
        delete vehicle_status_monitor;
    }


};

TEST_F(VehicleStatusMonitorNodeTest, test_pub_sub){
    //need time for everything to get set up
    ros::Duration d(1);
    d.sleep();

    EXPECT_EQ(1,pub.getNumSubscribers());
    EXPECT_EQ(1,sub.getNumPublishers());
}

/** VP Test 7
 *
 */
TEST_F(VehicleStatusMonitorNodeTest, test_in_motion_transition){
    //need time for everything to get set up
    ros::Duration d(1);
    d.sleep();
    handle.setParam("/vehicle_status_monitor/in_motion_threshold", 1.0);
    vehicle_status_monitor->load_params();

    geometry_msgs::Twist tw;
    tw.linear.x = vehicle_status_monitor->get_in_motion_threshold() + 0.1;

    pub.publish(tw);

    ASSERT_TRUE(wait_for_message(0.5));
    EXPECT_EQ("in_motion", received.data);
}

/** VP Test 8
 *
 */
TEST_F(VehicleStatusMonitorNodeTest, test_stopped_transition){
    //need time for everything to get set up
    ros::Duration d(1);
    d.sleep();

    geometry_msgs::Twist tw;
    tw.linear.x = vehicle_status_monitor->get_in_motion_threshold() + 0.1;

    pub.publish(tw);

    ASSERT_TRUE(wait_for_message(0.5));
    EXPECT_EQ("in_motion", received.data);

    geometry_msgs::Twist tw2;
    pub.publish(tw2);

    ASSERT_TRUE(wait_for_message(0.5));
    EXPECT_EQ( "stopped", received.data);
}


/** VP Test 9
 *
 */
TEST_F(VehicleStatusMonitorNodeTest, test_started_transition){
    //need time for everything to get set up
    ros::Duration d(1);
    d.sleep();

    geometry_msgs::Twist tw;
    tw.linear.x = 0.0;

    pub.publish(tw);

    ASSERT_TRUE(wait_for_message(0.5));
    EXPECT_EQ(received.data, "started");
}






int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_status_monitor_verifications");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}