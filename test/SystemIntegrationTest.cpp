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

Description: contains tests which can evaluate the contents of system_verification.bag generated for the system integration test

*/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "MeisterCommon.h"

using namespace meister::CommonStrings;
using namespace meister::TestUtilities;

/** test fixture for verifying the StateUpdate class;
 *
 *
class System_Integration_Test : public ::testing::Test {
public:
    ros::NodeHandle handle;
    std::string system_verification_bag;

    System_Integration_Test(): handle(){
        handle.param("integration_test_bagfile",system_verification_bag, SYSTEM_VERIFICATION_BAG);
    }


};
*/

std::string system_verification_bag = SYSTEM_VERIFICATION_BAG; //!< if argument bagfile=<pathspec> is passed into this test, this will be set to <pathspec>

TEST(simple, test){
    EXPECT_TRUE(false) << "file path: " << system_verification_bag;
}
/** Verification Plan Test 1
 *
 * This test evaluates the contents of the system_verification.bag file. It verifies messages were being published to
 * the “/meister/filtered_lidar_points” topic. Then, for each message it finds the “/catvehicle/lidar_points” message
 * with the same timestamp and verifies that the “/meister/filtered_lidar_points” message contains fewer points than the
 * original.
 */
TEST(System_Integration_Test, lidar_filter){
    rosbag::Bag bag;
    bag.open(system_verification_bag, rosbag::bagmode::Read);

    auto lidar_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(CATVEHICLE_LIDAR_TOPIC, bag);
    auto filtered_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(LIDAR_FILTER_OUTPUT_TOPIC, bag);

    // check that filtered point clouds were published
    ASSERT_GT(filtered_point_clouds.size(), 0);

    for (auto fpc : filtered_point_clouds){
        bool matches = false;

        for (auto pc : lidar_point_clouds){
            if (fpc.header.stamp == pc.header.stamp){
                matches = true;
                // Check that the filtered point cloud contains fewer points than the original
                EXPECT_LT(fpc.points.size(), pc.points.size());
                break;
            }
        }

        // Check that point clouds published by the state update node correspond to point clouds published by the Lidar Filter node
        EXPECT_TRUE(matches) << "Filtered Point Cloud: " << fpc << " does not have a corresponding filtered point cloud.";
    }

    bag.close();

}

/** Verification Plan Test 5
 *
 * System_Integration_Test.state_update evaluates the contents of system_verification.bag. It ensures that for each
 * point cloud published by the state update node, a corresponding point cloud was published by the lidar filter node.
 * Further, it verifies that the frame id of the point clouds published by the State Update Node is “/catvehicle/odom”
 * the global coordinate frame for the simulation.
 *
 */
TEST(System_Integration_Test, state_update){
    rosbag::Bag bag;
    bag.open(system_verification_bag, rosbag::bagmode::Read);

    auto transformed_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(STATE_UPDATE_OUTPUT_TOPIC, bag);
    auto filtered_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(LIDAR_FILTER_OUTPUT_TOPIC, bag);

    for (auto tpc : transformed_point_clouds){
        bool matches = false;

        //Check that point clouds published by the State Update Node are in the expected frame
        EXPECT_EQ(tpc.header.frame_id, CATVEHICLE_ROOT_COORDINATE_FRAME);

        for (auto fpc : transformed_point_clouds){
            if (fpc.header.stamp == tpc.header.stamp){
                matches = true;
                break;
            }
        }

        // Check that point clouds published by the state update node correspond to point clouds published by the Lidar Filter node
        EXPECT_TRUE(matches) << "Transformed Point Cloud: " << tpc << " does not have a corresponding filtered point cloud.";
    }

    bag.close();
}


/** Verification Plan Test 10
 *
 * System_Integration_Test.obstacle_detector parses the bagfile created during test setup to ensure that the messages
 * published by the Obstacle Detector Node to the “/detections” topic are correlated with the messages published to the
 * “meister/transformed_lidar_points” topic by the state update node. The test verifies that there will be at least one
 * “detection” polygon published for every non-empty point cloud published by the state update node. Furthermore, the
 * test verifies that every point within a point cloud published by state update lies within the bounds of one of the
 * “detection” polygons.
 *
 */
TEST(System_Integration_Test, obstacle_detector){
    rosbag::Bag bag;
    bag.open(system_verification_bag, rosbag::bagmode::Read);

    ros::Time stoped_time(0,0);

    // get time at which vehicle stopped. This is used as a cutoff for the data analyzed from the bagfile
    // because I cannot know what order the nodes will shutdown in if either the Laser Data node or the State Update node
    // shutdown after the obstacle detector, it generates a false failure in this test
    for (auto m : read_all_message_instances_from_topic(VEHICLE_STATUS_OUTPUT_TOPIC, bag)){
        std_msgs::StringConstPtr status = m.instantiate<std_msgs::String>();

        if (status != NULL && status->data == "stopped"){
            if (m.getTime() > stoped_time)
                stoped_time = m.getTime();
        }
    }

    auto lidar_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(STATE_UPDATE_OUTPUT_TOPIC, bag, stoped_time);
    auto detections = read_all_messages_from_topic<geometry_msgs::PolygonStamped>(DETECTION_TOPIC, bag);

    for (auto pc : lidar_point_clouds){
        if (pc.points.size() > 0) {

            std::vector<geometry_msgs::PolygonStamped> lidar_detections;

            for (auto detection : detections){
                if (detection.header.stamp == pc.header.stamp){
                    lidar_detections.push_back(detection);
                }
            }

            // check that for every non-empty transformed lidar point cloud produced by the laser data node, there is a detection published
            ASSERT_GT(lidar_detections.size() , 0) << "Missing Detection for Point Cloud at: "  << pc.header.stamp;

            for (auto point : pc.points){
                bool in_detection = false;

                for (auto detection : lidar_detections){
                    if (!in_detection){
                        in_detection = point_in(point, detection);
                    }
                }

                // check that every point in every transformed lidar point cloud is contained in an associated detection
                EXPECT_TRUE(in_detection) << "Point: " << point << " is not contained within a detection polygon";
            }
        }
    }

    bag.close();
}

/** Verification Plan Test 12
 *
 * System_Integration_Test.detections_are_aabb parses the bagfile created during test set up to verify that every
 * detection published by the Obstacle Detector Node is in the “/catvehicle/odom” coordinate frame, that every detection
 * polygon consists of exactly 4 vertices, and that the edges of each polygon are aligned with either the x or y axis.
 */
TEST(System_Integration_Test, detections_are_aabb){
    rosbag::Bag bag;
    bag.open(system_verification_bag, rosbag::bagmode::Read);

    std::vector<std::string> topics;

    topics.push_back(STATE_UPDATE_OUTPUT_TOPIC);

    topics.push_back(DETECTION_TOPIC);

    rosbag::View view2(bag, rosbag::TopicQuery(topics));

    int detection_count = 0;

    // get all of the detections published by the obstacle detector node
    BOOST_FOREACH(rosbag::MessageInstance const m, view2){
        geometry_msgs::PolygonStamped::ConstPtr ps = m.instantiate<geometry_msgs::PolygonStamped>();
        if (ps != NULL){
            detection_count++;

            // verify polygon is a rectangle
            EXPECT_EQ(4, ps->polygon.points.size());

            // verify polygon is in the catvehicle's global coordinate system
           EXPECT_EQ(CATVEHICLE_ROOT_COORDINATE_FRAME, ps->header.frame_id);
            auto points = ps->polygon.points;

            // verify that edges of the polygon are parallel to either the X or Y axis
            EXPECT_EQ(points[0].y, points[1].y);
            EXPECT_EQ(points[3].y, points[2].y);
            EXPECT_EQ(points[0].x, points[3].x);
            EXPECT_EQ(points[1].x, points[2].x);
        }
    }

    // verify that at least one polygon was evaluated
    ASSERT_GT(detection_count, 0) << " no detections in bagfile to evaluate!";
}

/** Verification Plan Test 16
 *
 * Test 16 is a unit test of the “system_verification.bag” file created during test set up. It verifies that for each
 * polygon published to the “/detections” topic there is either a PointCloud published to the
 * “/meister/transformed_lidar_points” topic or the “/meister/laser_data” topic with the corresponding timestamp, thus
 * indicating that message was the source of the detection. It verifies that one or more polygons is attributable to
 * each PointCloud topic.
 *
 */
TEST(System_Integration_Test, laser_and_lidar_detections){
    rosbag::Bag bag;
    bag.open(system_verification_bag, rosbag::bagmode::Read);

    ros::Time stoped_time(0,0);

    // get time at which vehicle stopped. This is used as a cutoff for the data analyzed from the bagfile
    // because I cannot know what order the nodes will shutdown in if either the Laser Data node or the State Update node
    // shutdown after the obstacle detector, it generates a false failure in this test
    for (auto m : read_all_message_instances_from_topic(VEHICLE_STATUS_OUTPUT_TOPIC, bag)){
        std_msgs::StringConstPtr status = m.instantiate<std_msgs::String>();

        if (status != NULL && status->data == "stopped"){
            if (m.getTime() > stoped_time)
                stoped_time = m.getTime();
        }
    }

    auto laser_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(LASER_DATA_OUTPUT_TOPIC, bag, stoped_time);
    auto lidar_point_clouds = read_all_messages_from_topic<sensor_msgs::PointCloud>(STATE_UPDATE_OUTPUT_TOPIC, bag, stoped_time);
    auto detections = read_all_messages_from_topic<geometry_msgs::PolygonStamped>(DETECTION_TOPIC, bag);

    for (auto pc : lidar_point_clouds){
        if (pc.points.size() > 0) {

            std::vector<geometry_msgs::PolygonStamped> lidar_detections;

            for (auto detection : detections){
                if (detection.header.stamp == pc.header.stamp){
                    lidar_detections.push_back(detection);
                }
            }

            // check that for every non-empty transformed lidar point cloud produced by the laser data node, there is a detection published
            ASSERT_GT(lidar_detections.size() , 0) << "Missing Detection for Point Cloud at: "  << pc.header.stamp;

            for (auto point : pc.points){
                bool in_detection = false;

                for (auto detection : lidar_detections){
                    if (!in_detection){
                        in_detection = point_in(point, detection);
                    }
                }

                // check that every point in every transformed lidar point cloud is contained in an associated detection
                EXPECT_TRUE(in_detection) << "Point: " << point << " is not contained within a detection polygon";
            }
        }
    }


    for (auto pc : laser_point_clouds){
        if (pc.points.size() > 0) {

            std::vector<geometry_msgs::PolygonStamped> laser_detections;

            for (auto detection : detections){
                if (detection.header.stamp == pc.header.stamp){
                    laser_detections.push_back(detection);
                }
            }
            // check that for every non-empty laser point cloud produced by the laser data node, there is a detection published
            ASSERT_GT(laser_detections.size() , 0) << "Missing Detection for Point Cloud at: "  << pc.header.stamp;

            for (auto point : pc.points){
                bool in_detection = false;

                for (auto detection : laser_detections){
                    if (!in_detection){
                        in_detection = point_in(point, detection);
                    }
                }

                // check that every point in every laser point cloud is contained in an associated detection
                EXPECT_TRUE(in_detection) << "Point: " << point << " is not contained within a detection polygon";
            }
        }
    }


    bag.close();
}

int main(int argc, char **argv) {
    //ros::init(argc, argv, "system_verifications");
    ::testing::InitGoogleTest(&argc, argv);

    // allow use of rostest without requiring roscore when using rosrun
    if (argc > 1){
        for (int i = 1; i < argc; i++){
           if (boost::starts_with(argv[i], "bagfile=")){
               system_verification_bag = std::string(argv[i]).substr(8);
               break;
           }
        }
    }

    return RUN_ALL_TESTS();
}
