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


 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <vector>
#include <geometry_msgs/PolygonStamped.h>
#include "RosCommunications.h"
#include "ObstacleDetector.h"
#include <ctime>
#include "MeisterCommon.h"
#include "SDFModel.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>

using namespace std;
using namespace meister;
using namespace meister::CommonStrings;


class ObstacleDetectorNodeTest : public ::testing::Test {
public:
    ros::NodeHandle handle;
    ros::Publisher lidar_pub;
    ros::Publisher laser_pub;
    ros::Publisher status_pub;
    ros::Subscriber detections_sub;

    ObstacleDetector *det;
    boost::thread *thread;


    vector<geometry_msgs::PolygonStamped> received;

    int r_count;

    void callback(const geometry_msgs::PolygonStamped &pc){
        r_count++;
        received.push_back(pc);
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

    void wait_for_message(){
        int last = r_count;

        while(r_count == last){
            ros::spinOnce();
        }
    }
    /** create a vector containing an evenly distributed grid of geometry_messages::Point32
     *
     * @param xi starting x position
     * @param yi starting y position
     * @param zi z value of all points
     * @param spacing spacing between points
     * @param cols number of columns
     * @param rows number of rows
     * @return vector containing the generated points
     */
    vector<geometry_msgs::Point32> create_point_grid(double xi, double yi, double zi, double spacing, int cols, int rows){
        vector <geometry_msgs::Point32> points;
        double x = xi;
        double y = yi;
        geometry_msgs::Point32 point;


        for (int row = 0; row < rows; row++){
            for (int col = 0; col < cols; col++){
                point.x = x;
                point.y = y;
                point.z = zi;
                points.push_back(point);
                x += spacing;
            }

            y += spacing;
            x = xi;
        }

        return points;
    }

    ObstacleDetectorNodeTest() : handle("test_node") {

        laser_pub = handle.advertise<sensor_msgs::PointCloud>(LASER_DATA_OUTPUT_TOPIC,100);
        lidar_pub = handle.advertise<sensor_msgs::PointCloud>(STATE_UPDATE_OUTPUT_TOPIC,100);
        status_pub = handle.advertise<std_msgs::String>(VEHICLE_STATUS_OUTPUT_TOPIC, 100);
        detections_sub = handle.subscribe(DETECTION_TOPIC,1000,&ObstacleDetectorNodeTest::callback, this);

        // instantiate an ObstacleDetector and start it running
        det = new ObstacleDetector();
        thread = new boost::thread(boost::bind(&ObstacleDetector::run, det));
    }

    ~ObstacleDetectorNodeTest(){
        // shut down the obstacle detector
        det->stop();
        thread->join();
        delete thread;
        delete det;
    }


};

TEST_F(ObstacleDetectorNodeTest, test_pub_sub){
        ros::Duration d(1);
    d.sleep();

    EXPECT_GT(lidar_pub.getNumSubscribers(), 0) << "nothing subscribed to /meister/transformed_lidar_points";
    EXPECT_GT(status_pub.getNumSubscribers(), 0) << "nothing subscribed to /meister/vehicle_status";
    EXPECT_GT(detections_sub.getNumPublishers(), 0) << "nothing publishing to /detections";
    EXPECT_LE(detections_sub.getNumPublishers(), 1) << "too many nodes publishing to /detections";



}

TEST_F(ObstacleDetectorNodeTest, test_publish_detection){
    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    pc.points = create_point_grid(0.3, 0.3, 1, 0.5, 3, 3);

    lidar_pub.publish(pc);

    EXPECT_TRUE(wait_for_message(1));
    EXPECT_EQ(1, received.size());
}


TEST_F(ObstacleDetectorNodeTest, test_laser_publish_detection){
    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    pc.points = create_point_grid(0.3, 0.3, 1, 0.5, 3, 3);

    laser_pub.publish(pc);

    EXPECT_TRUE(wait_for_message(1));
    EXPECT_EQ(1, received.size());
}

/** VP Test 11
 *
 */
TEST_F(ObstacleDetectorNodeTest, test_points_in_detection){
    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    pc.points = create_point_grid(1, 1, 1, 0.5, 3, 3);

    lidar_pub.publish(pc);

    EXPECT_FALSE(wait_for_messages(1, 9));
    ASSERT_EQ(1, received.size());

    for (std::vector<geometry_msgs::Point32>::iterator it = pc.points.begin(); it != pc.points.end(); ++it){
        auto min_x = received[0].polygon.points[0].x;
        auto max_x = received[0].polygon.points[2].x;
        auto min_y = received[0].polygon.points[0].y;
        auto max_y = received[0].polygon.points[2].y;

        ASSERT_TRUE(it->x <= max_x && it->x >= min_x);
        ASSERT_TRUE(it->y <= max_y && it->y >= min_y);
    }
}

/** VP Test 12
 *
 */
TEST_F(ObstacleDetectorNodeTest, test_is_aabb){
    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    pc.points = create_point_grid(0.3, 0.3, 1, 0.5, 3, 3);

    lidar_pub.publish(pc);

    EXPECT_TRUE(wait_for_message(1));
    EXPECT_EQ(1, received.size());
    EXPECT_EQ(4, received[0].polygon.points.size()) << "detection polygon does not have 4 verticies";

    auto points = received[0].polygon.points;

    EXPECT_EQ(points[0].y, points[1].y);
    EXPECT_EQ(points[3].y, points[2].y);
    EXPECT_EQ(points[0].x, points[3].x);
    EXPECT_EQ(points[1].x, points[2].x);

}

/** VP Test 13
 *
 */
TEST_F(ObstacleDetectorNodeTest, test_world_creation_timing){
    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    int rows = 10;
    int cols = 10;
    double spacing = 5.0;
    double xi = 0.25;
    double yi = 0.25;

    ros::Rate r(10);
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            pc.points = create_point_grid(xi + (i * spacing), yi + (j * spacing), 1, 0.01, 100, 100);
            lidar_pub.publish(pc);
            EXPECT_TRUE(wait_for_message(1));
            r.sleep();

        }
    }

    ros::Time start = ros::Time::now();
    det->create_world();
    ros::Time end = ros::Time::now();

    EXPECT_LE( (end - start).toSec(), 60.0);

}


TEST_F(ObstacleDetectorNodeTest, test_world_creation_trigger_and_timing){
    using namespace meister::CommonStrings;
    std::string output_file;
    ASSERT_TRUE(handle.getParam(OUTPUT_WORLD_FILENAME_PARAMETER, output_file)) << OUTPUT_WORLD_FILENAME_PARAMETER << " not set, could not complete test!";

    if (boost::filesystem::exists(output_file)){
        boost::filesystem::remove(output_file);
    }

    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    int rows = 10;
    int cols = 10;
    double spacing = 5.0;
    double xi = 0.25;
    double yi = 0.25;

    ros::Rate r(10);
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            pc.points = create_point_grid(xi + (i * spacing), yi + (j * spacing), 1, 0.01, 100, 100);
            lidar_pub.publish(pc);
            EXPECT_TRUE(wait_for_message(1));
            r.sleep();

        }
    }

    std_msgs::String status;
    status.data = "stopped";

    ros::Time start = ros::Time::now();
    status_pub.publish(status);
    ros::Rate r2(100);
    while (!boost::filesystem::exists(output_file) && (ros::Time::now() - start).toSec() < 60.0){
        r2.sleep();
    }
    ros::Time end = ros::Time::now();
    ASSERT_TRUE(boost::filesystem::exists(output_file));
    EXPECT_LE( (end - start).toSec(), 60.0);

}

/** Test 14
 *
 */
TEST_F(ObstacleDetectorNodeTest, test_world_contents){
    using boost::property_tree::ptree;
    using namespace boost::algorithm;
    using namespace meister::CommonStrings;

    std::string output_file;
    ASSERT_TRUE(handle.getParam(OUTPUT_WORLD_FILENAME_PARAMETER, output_file)) << OUTPUT_WORLD_FILENAME_PARAMETER << " not set, could not complete test!";

    if (boost::filesystem::exists(output_file)){
        boost::filesystem::remove(output_file);
    }


    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    int rows = 10;
    int cols = 10;
    double spacing = 5.0;
    double xi = 0.25;
    double yi = 0.25;
    int detection_count = 0;

    ros::Rate r(10);
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            pc.points = create_point_grid(xi + (i * spacing), yi + (j * spacing), 2, 0.01, 100, 100);
            lidar_pub.publish(pc);
            EXPECT_TRUE(wait_for_message(1));
            r.sleep();

        }
    }

    std_msgs::String status;
    status.data = "stopped";

    //ros::Time start = ros::Time::now();
    status_pub.publish(status);
    ros::Duration d2(60);
    d2.sleep();
    //ros::Time end = ros::Time::now();
    ASSERT_TRUE(boost::filesystem::exists(output_file));


    ptree world_tree;
    std::vector<ptree> models;

    ASSERT_TRUE(handle.getParam(OUTPUT_WORLD_FILENAME_PARAMETER, output_file)) << OUTPUT_WORLD_FILENAME_PARAMETER << " not set, could not complete test!";
    d.sleep(); // keeps the next item from throwing randomly
    ASSERT_NO_THROW(read_xml(output_file, world_tree, boost::property_tree::xml_parser::trim_whitespace));

    BOOST_FOREACH(const ptree::value_type &v, world_tree.get_child("sdf.world")){

      if (v.first == "model" && starts_with(v.second.get<std::string>("<xmlattr>.name","fail"),"aabb")){
          models.push_back(v.second);
      }

    }

    ASSERT_EQ(models.size(), rows * cols);
}

/** VP Test 15
 *
 */
TEST_F(ObstacleDetectorNodeTest, test_bounding_volume_dimensions){
    using boost::property_tree::ptree;
    using namespace boost::algorithm;
    using namespace meister::CommonStrings;

    std::string output_file;
    ASSERT_TRUE(handle.getParam(OUTPUT_WORLD_FILENAME_PARAMETER, output_file)) << OUTPUT_WORLD_FILENAME_PARAMETER << " not set, could not complete test!";

    if (boost::filesystem::exists(output_file)){
        boost::filesystem::remove(output_file);
    }

    ros::Duration d(1);
    d.sleep();

    sensor_msgs::PointCloud pc;
    pc.header.frame_id = CATVEHICLE_ROOT_COORDINATE_FRAME;
    pc.header.stamp = ros::Time::now();

    pc.points = create_point_grid(0.3, 0.3, 1, 0.5, 3, 3);

    pc.points[3].z = 2; // change the height of one point to verify the height later

    lidar_pub.publish(pc);

    EXPECT_TRUE(wait_for_message(1));

    std_msgs::String status;
    status.data = "stopped";

    status_pub.publish(status);
    ros::Duration d2(60);
    d2.sleep();

    ASSERT_TRUE(boost::filesystem::exists(output_file));

    ptree world_tree;
    std::vector<ptree> models;

    ASSERT_TRUE(handle.getParam(OUTPUT_WORLD_FILENAME_PARAMETER, output_file)) << OUTPUT_WORLD_FILENAME_PARAMETER << " not set, could not complete test!";
    ASSERT_NO_THROW(read_xml(output_file, world_tree, boost::property_tree::xml_parser::trim_whitespace));

    BOOST_FOREACH(const ptree::value_type &v, world_tree.get_child("sdf.world")){

        if (v.first == "model" && starts_with(v.second.get<std::string>("<xmlattr>.name","fail"),"aabb")){
            models.push_back(v.second);
        }

    }

    ASSERT_EQ(models.size(), 1);

    SDFModel model(models[0]);

    ASSERT_EQ(model.height, pc.points[3].z);

    for (auto it: pc.points){
        geometry_msgs::Point32 pt = it;
        EXPECT_TRUE(model.in_model(pt));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_detector_test");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}