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
Contains tests intended to verify the requirments of the State Update Node
 see README or verification plan for more detail


 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "MeisterCommon.h"
#include "StateUpdate.h"

using namespace meister::CommonStrings;

/** test fixture for verifying the StateUpdate class;
 *
 */
class StateUpdateNodeTest : public ::testing::Test {
public:
    StateUpdate *state_update_node;
    boost::thread *thread;

    ros::NodeHandle handle;
    ros::Publisher lidar_pub;
    ros::Subscriber transformed_sub;

    sensor_msgs::PointCloud received;

    int r_count;

    /** callback function for the ros::Subscriber
     *
     * @param pc the poinclound received by the ros::Subscriber
     */
    void callback(const sensor_msgs::PointCloud &pc){
        r_count++;
        received = pc;
    }

    /** function to wait for transformed_pub to receive a message
     *
     * @param timeout number of seconds to wait
     * @return boolean indicating if a message was received before timeout
     */
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

    /** function to wait as long as it takes for transformed_pub to receive a message
     *
     */
    void wait_for_message(){
        int last = r_count;

        while(r_count == last){
            ros::spinOnce();
        }
    }

    /** test fixture constructor
     *
     *  instantiates a StateUpdate Node and starts it running in a separate thread.
     *  then instantiates a publisher to provide input messages to the StateUpdate node
     *  Finally, it instantiates a subscriber to receive output messages from the StateUpdate node
     *
     */
    StateUpdateNodeTest() : handle("state_update_node_test"){

        lidar_pub = handle.advertise<sensor_msgs::PointCloud>(LIDAR_FILTER_OUTPUT_TOPIC, 100);
        transformed_sub = handle.subscribe(STATE_UPDATE_OUTPUT_TOPIC, 100, &StateUpdateNodeTest::callback, this);

        state_update_node = new StateUpdate();
        thread = new boost::thread(boost::bind(&StateUpdate::run, state_update_node));
    }

    /** Test Fixture Destructor
     *
     *  shuts down the State Update Node and frees memory allocated for it
     *
     */
    ~StateUpdateNodeTest(){
        state_update_node->stop();
        thread->join();
        delete thread;
        delete state_update_node;
    }

};

/** Test function to verify the the StateUpdate node subscribese and publishes to the expected topics
 *
 */
TEST_F(StateUpdateNodeTest, test_pub_sub){
    // allow time for communication to be set up
    ros::Duration d(1);
    d.sleep();

    EXPECT_EQ(1, lidar_pub.getNumSubscribers());
    EXPECT_EQ(1, transformed_sub.getNumPublishers());

}


/**VP Test 6
 * test function to verify that the state update node correctly transforms a point cloud given a pure translational relationship
 *  between the root and /catvehicle/velodyne_link coordinate systems
 *
 *  First, the function publishes a point cloud in the /catvehicle/velodyne_link frame
 *  Second, it publishes a translational transform from the /catvehicle/velodyne_link to the /catvehicle/odom coordinate
 *  system.
 *  Lastly, it verifies that the point cloud published by the state update node has been translated appropriately
 *
 */
TEST_F(StateUpdateNodeTest, test_point_cloud_transform_translational){
    // allow time for ros node communication set up
    ros::Duration d(1);
    d.sleep();

    EXPECT_EQ(1, lidar_pub.getNumSubscribers());
    EXPECT_EQ(1, transformed_sub.getNumPublishers());

    tf::TransformBroadcaster br;
    sensor_msgs::PointCloud pc;
    ros::Time time = ros::Time::now();

    pc.header.frame_id = LIDAR_COORDINATE_FRAME;
    pc.header.stamp = time;

    geometry_msgs::Point32 point;

    point.x = 1;
    point.y = 1;
    point.z = 1;

    pc.points.push_back(point);

    lidar_pub.publish(pc);

    tf::Transform trf;
    //trf.setOrigin(tf::Vector3(0, 0, 0));
    //br.sendTransform(tf::StampedTransform(trf, time, "world", CATVEHICLE_ROOT_COORDINATE_FRAME));

    trf.setOrigin(tf::Vector3(1, 1, 1));
    br.sendTransform(tf::StampedTransform(trf, time, LIDAR_COORDINATE_FRAME ,CATVEHICLE_ROOT_COORDINATE_FRAME));


    ASSERT_TRUE(wait_for_message(1.0));
    ASSERT_EQ(1, received.points.size());
    ASSERT_EQ(0.0, received.points[0].x);
    ASSERT_EQ(0.0, received.points[0].y);
    ASSERT_EQ(0.0, received.points[0].z);

}

TEST_F(StateUpdateNodeTest, test_point_cloud_transform_rotational){
    // allow time for ros node communication set up
    ros::Duration d(1);
    d.sleep();

    EXPECT_EQ(1, lidar_pub.getNumSubscribers());
    EXPECT_EQ(1, transformed_sub.getNumPublishers());

    tf::TransformBroadcaster br;
    sensor_msgs::PointCloud pc;
    ros::Time time = ros::Time::now();

    pc.header.frame_id = LIDAR_COORDINATE_FRAME;
    pc.header.stamp = time;

    geometry_msgs::Point32 point;

    point.x = 0;
    point.y = 1;
    point.z = 1;

    pc.points.push_back(point);

    lidar_pub.publish(pc);

    tf::Transform trf;
    trf.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI/2);
    trf.setRotation(q);
    br.sendTransform(tf::StampedTransform(trf, time, LIDAR_COORDINATE_FRAME ,CATVEHICLE_ROOT_COORDINATE_FRAME));


    ASSERT_TRUE(wait_for_message(1.0));
    ASSERT_EQ(1, received.points.size());
    ASSERT_EQ(1.0, received.points[0].x);
    ASSERT_LT(received.points[0].y, 0.00001); // angular transforms aren't exact
    ASSERT_EQ(1.0, received.points[0].z);

}

TEST_F(StateUpdateNodeTest, test_point_cloud_transform_translational_rotational){
    // allow time for ros node communication set up
    ros::Duration d(1);
    d.sleep();

    EXPECT_EQ(1, lidar_pub.getNumSubscribers());
    EXPECT_EQ(1, transformed_sub.getNumPublishers());

    tf::TransformBroadcaster br;
    sensor_msgs::PointCloud pc;
    ros::Time time = ros::Time::now();

    pc.header.frame_id = LIDAR_COORDINATE_FRAME;
    pc.header.stamp = time;

    geometry_msgs::Point32 point;

    point.x = 0;
    point.y = 2;
    point.z = 2;

    pc.points.push_back(point);

    lidar_pub.publish(pc);

    tf::Transform trf;
    trf.setOrigin(tf::Vector3(1, 1, 1));
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI/2);
    trf.setRotation(q);
    br.sendTransform(tf::StampedTransform(trf, time, LIDAR_COORDINATE_FRAME ,CATVEHICLE_ROOT_COORDINATE_FRAME));


    ASSERT_TRUE(wait_for_message(1.0));
    EXPECT_EQ(1, received.points.size());
    EXPECT_EQ(1.0, received.points[0].x);
    EXPECT_EQ(1.0, received.points[0].y);
    EXPECT_EQ(1.0, received.points[0].z);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_update_test");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

