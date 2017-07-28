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
Contains Regression Tests For the Detection Grid Class
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "LaserDataProcessor.h"
#include "MeisterCommon.h"
#include <vector>
#include <math.h>

using namespace meister::CommonStrings;

/** Test fixture for the LaserDataProcessor class
 *
 */
class LaserDataNodeTest : public ::testing::Test {
public:
    ros::NodeHandle handle;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::shared_ptr<LaserDataProcessor> laser_data;
    std::shared_ptr<boost::thread> thread;

    sensor_msgs::PointCloud received;

    int r_count;

    void callback(const sensor_msgs::PointCloud &pc){
        r_count++;
        received = pc;
    }

    bool wait_for_message(double timeout){
        int last = r_count;
        int spinCount = 0;
        ros::Rate r(10);

        while(r_count == last && spinCount < timeout * 10){
            ros::spinOnce();
            r.sleep();
            spinCount++;
        }

        return r_count == last + 1;
    }

    LaserDataNodeTest() : handle("~") {
        pub = handle.advertise<sensor_msgs::LaserScan>(CATVEHICLE_LASER_SCAN_TOPIC,100);
        sub = handle.subscribe(LASER_DATA_OUTPUT_TOPIC,100,&LaserDataNodeTest::callback, this);

        laser_data.reset(new LaserDataProcessor());
        thread.reset(new boost::thread(boost::bind(&LaserDataProcessor::run, laser_data)));
    }

    ~LaserDataNodeTest(){
        laser_data->stop();
        thread->join();
    }


};

/** Test to see if LaserDataProcessor subcribes to /catvehicle/front_laser_points and publishes to /meister/laser_points
 *  excludes the coordinate system transform
 */
TEST_F(LaserDataNodeTest, test_pub_sub){
    // wait for Ros communications to initialize
    ros::Duration d(1);
    d.sleep();

    EXPECT_EQ(1,pub.getNumSubscribers());
    EXPECT_EQ(1,sub.getNumPublishers());

};

/**Test 17
 *
 */
TEST_F(LaserDataNodeTest, test_laser_to_point_cloud){
    // wait for Ros communications to initialize
    ros::Duration d(0.5);
    d.sleep();

    // create a laser scan from a point cloud where the expected x coord of every point is 10

    sensor_msgs::LaserScan ls;

    ls.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    ls.header.stamp = ros::Time::now();

    ls.angle_min = -M_PI/4;
    ls.angle_max = M_PI/4;
    ls.angle_increment = M_PI/10;
    ls.time_increment = 0.0;
    ls.range_min = 1.5;
    ls.range_max = 80.0;

    for (int i = 0; i < (ls.angle_max - ls.angle_min) / ls.angle_increment; i++){
        float angle = ls.angle_min + (i * ls.angle_increment);
        auto r = 10.0 / cos(angle);
        //ADD_FAILURE() << "angle = " << angle;
        //ADD_FAILURE() << "cos(" << angle << ") = " << cos(angle);
        //ADD_FAILURE() << "radius = " << r;
        ls.ranges.push_back(r);
    }

    // publish the laser scan to /catvehicle/front_laser_points
    pub.publish(ls);

    //check that a message has been published after 0.5 seconds
    ASSERT_TRUE(wait_for_message(0.5));

    //check that the x value of every point is 10
    for (auto pt : received.points){
        EXPECT_EQ(10.0, pt.x);
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_data_verifications");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}