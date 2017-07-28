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
Contains tests intended to verify the following requirements have been met

 Lidar Filter Node
1. (B) The Lidar Filter Node shall produce a filtered dataset from the catvehicle/lidar_points topic
1.1. (B) A distance threshold filter shall be used to exclude objects less than 1.5 m from the velodyne sensor in order to exclude the hood of the car
1.2. (B) A distance threshold filter shall be used to exclude the points returned from the extents of the /catvehicle/lidar_points topic.
1.3. (B) A height threshold filter shall be used to exclude the ground or near ground (< 0.2m) detections
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Polygon.h>
#include <boost/thread.hpp>
#include "LidarFilter.h"
#include "MeisterCommon.h"

using namespace meister::CommonStrings;


class LidarFilterNodeTest : public ::testing::Test {
public:
    ros::NodeHandle handle;
    ros::Publisher pub;
    ros::Subscriber sub;
    LidarFilter *lidar_filter;
    boost::thread *thread;

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

    LidarFilterNodeTest() : handle("~") {
        pub = handle.advertise<sensor_msgs::PointCloud>("/catvehicle/lidar_points",1000);
        sub = handle.subscribe("/meister/filtered_lidar_points",1000,&LidarFilterNodeTest::callback, this);

        lidar_filter = new LidarFilter();
        thread = new boost::thread(boost::bind(&LidarFilter::run, lidar_filter));
    }

    ~LidarFilterNodeTest(){
        lidar_filter->stop();
        thread->join();

        delete lidar_filter;
        delete thread;
    }


};



/*
TEST(LidarFilterNodeTest, test_subscribe_and_publish){
    ros::NodeHandle handle;
    ros::Publisher pub;
    pub = handle.advertise<sensor_msgs::PointCloud>("/catvehicle/lidar_points",1000);
    //ros::Subscriber sub = handle.subscribe<sensor_msgs::PointCloud>("/catvehicle/lidar_points",1000,callback);

    ros::Rate r(1);
    r.sleep();

    ROS_DEBUG("WTF MATES");
    EXPECT_EQ(1,pub.getNumSubscribers());

}
*/


TEST_F(LidarFilterNodeTest, test_pub_sub){
    // necessary to give everything time to connect for some reason;
    ros::Rate r(1);
    r.sleep();

    EXPECT_EQ(1,pub.getNumSubscribers());
    EXPECT_EQ(1,sub.getNumPublishers());

    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    cloud.header.stamp = ros::Time::now();

    pt.x = 10;
    pt.y = 10;
    pt.z = 10;

    cloud.points.push_back(pt);

    pub.publish(cloud);

    ASSERT_TRUE(wait_for_message(1));

    ASSERT_EQ(received.points.size(), cloud.points.size());
    EXPECT_EQ(received.points[0].x, cloud.points[0].x);
    EXPECT_EQ(received.points[0].y, cloud.points[0].y);
    EXPECT_EQ(received.points[0].z, cloud.points[0].z);

}
/** VP Test 2
 *
 */
TEST_F(LidarFilterNodeTest, filter_close_points){
    // necessary to give everything time to connect for some reason;
    ros::Rate r(1);
    r.sleep();

    EXPECT_EQ(1,pub.getNumSubscribers());
    EXPECT_EQ(1,sub.getNumPublishers());

    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    cloud.header.stamp = ros::Time::now();

    pt.x = 10;
    pt.y = 10;
    pt.z = 10;

    cloud.points.push_back(pt);

    pt.x = 1;
    pt.y = 1;
    pt.z = 1;

    cloud.points.push_back(pt);

    pub.publish(cloud);

    ASSERT_TRUE(wait_for_message(1));

    ASSERT_EQ(received.points.size(), 1);
    EXPECT_EQ(received.points[0].x, cloud.points[0].x);
    EXPECT_EQ(received.points[0].y, cloud.points[0].y);
    EXPECT_EQ(received.points[0].z, cloud.points[0].z);

}

/** VP Test 3
 *
 */
TEST_F(LidarFilterNodeTest, filter_far_points){
    // necessary to give everything time to connect for some reason;
    ros::Rate r(1);
    r.sleep();

    EXPECT_EQ(1,pub.getNumSubscribers());
    EXPECT_EQ(1,sub.getNumPublishers());

    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    cloud.header.stamp = ros::Time::now();

    pt.x = 10;
    pt.y = 10;
    pt.z = 10;

    cloud.points.push_back(pt);

    pt.x = 45.0;
    pt.y = 0.0;
    pt.z = 1,0;

    cloud.points.push_back(pt);

    pub.publish(cloud);

    wait_for_message(1);

    ASSERT_EQ(received.points.size(), 1);
    EXPECT_EQ(received.points[0].x, cloud.points[0].x);
    EXPECT_EQ(received.points[0].y, cloud.points[0].y);
    EXPECT_EQ(received.points[0].z, cloud.points[0].z);

}

/** VP Test 4
 *
 */
TEST_F(LidarFilterNodeTest, filter_ground_points){
    // necessary to give everything time to connect for some reason;
    ros::Rate r(1);
    r.sleep();

    EXPECT_EQ(1,pub.getNumSubscribers());
    EXPECT_EQ(1,sub.getNumPublishers());

    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    cloud.header.stamp = ros::Time::now();

    pt.x = 10;
    pt.y = 10;
    pt.z = 10;

    cloud.points.push_back(pt);

    pt.x = 10;
    pt.y = 10;
    pt.z = lidar_filter->get_min_height() - 0.01;

    cloud.points.push_back(pt);

    pub.publish(cloud);

    wait_for_message(1);

    ASSERT_EQ(received.points.size(), 1);
    EXPECT_EQ(received.points[0].x, cloud.points[0].x);
    EXPECT_EQ(received.points[0].y, cloud.points[0].y);
    EXPECT_EQ(received.points[0].z, cloud.points[0].z);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_filter_verifications");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
