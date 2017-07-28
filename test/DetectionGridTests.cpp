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
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include "DetectionGrid.h"
#include <vector>

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


// discovered that a grid containing only one point would cause a segfault while writing bounding_box_area_gt_zero test
TEST(DetectionGridRegressionTests, construct_grid_with_single_point){
    //create a point cloud containing a single point
    std::vector<geometry_msgs::Point32> pc;
    geometry_msgs::Point32 pt;
    pc.push_back(pt);

    //create DetectionGrid
    std::shared_ptr<DetectionGrid> grid;
    ASSERT_NO_FATAL_FAILURE(grid.reset(new DetectionGrid(pc, 1)));
}


// check that a bounding box will always have a greater than zero area
TEST(DetectionGridRegressionTests, bounding_box_area_gt_zero ){
    //create a point cloud containing a single point
    std::vector<geometry_msgs::Point32> pc;
    geometry_msgs::Point32 pt;
    pc.push_back(pt);

    //create DetectionGrid
    std::shared_ptr<DetectionGrid> grid;
    grid.reset(new DetectionGrid(pc, 1));

    vector<Point32> poly = grid->get_axis_aligned_bounding_box(1).points;

    geometry_msgs::Point32 p1 = poly[0];
    geometry_msgs::Point32 p2 = poly[2];

    double l = p2.x - p1.x;
    double w = p2.y - p1.y;

    EXPECT_GT((l * w), 0);
}

// verify that setting points with an empty vector will not cause a crash
TEST(DetectionGridRegressionTests, empty_points){
  std::vector<geometry_msgs::Point32> pc;

  //create DetectionGrid
  std::shared_ptr<DetectionGrid> grid;
  ASSERT_NO_FATAL_FAILURE(grid.reset(new DetectionGrid(1)));
  ASSERT_NO_FATAL_FAILURE(grid->setPoints(pc));

}

// integer value points on the outermost edge of the grid were causing it to throw an exception
// not the grid is a little larger than the max point
TEST(DetectionGridRegressionTests, edge_points){
    std::vector<geometry_msgs::Point32> pc = create_point_grid(1, 1, 1, 0.5, 3, 3);

    //create DetectionGrid
    std::shared_ptr<DetectionGrid> grid;
    ASSERT_NO_THROW(grid.reset(new DetectionGrid(0.5)));
    ASSERT_NO_THROW(grid->setPoints(pc));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "DetectionGridTest");
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
