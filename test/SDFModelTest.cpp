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
Contains regression tests for the SDFModel class used in regression testing the ObstacleDetector
*/

#include <gtest/gtest.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <sstream>
#include "SDFModel.h"

const std::string BOX_MODEL =
        "      <pose>5.0 6.0 7.0 0 0 0</pose>\n"
        "      <link name='link'>\n"
        "        <inertial>\n"
        "          <mass>1</mass>\n"
        "          <inertia>\n"
        "            <ixx>1</ixx>\n"
        "            <ixy>0</ixy>\n"
        "            <ixz>0</ixz>\n"
        "            <iyy>1</iyy>\n"
        "            <iyz>0</iyz>\n"
        "            <izz>1</izz>\n"
        "          </inertia>\n"
        "        </inertial>\n"
        "        <collision name='collision'>\n"
        "          <geometry>\n"
        "            <box>\n"
        "              <size>1.0 2.0 3.0</size>\n"
        "            </box>\n"
        "          </geometry>\n"
        "          <max_contacts>10</max_contacts>\n"
        "          <surface>\n"
        "            <bounce/>\n"
        "            <friction>\n"
        "              <ode/>\n"
        "            </friction>\n"
        "            <contact>\n"
        "              <ode/>\n"
        "            </contact>\n"
        "          </surface>\n"
        "        </collision>\n"
        "        <visual name='visual'>\n"
        "          <geometry>\n"
        "            <box>\n"
        "              <size>1 1 1</size>\n"
        "            </box>\n"
        "          </geometry>\n"
        "          <material>\n"
        "            <script>\n"
        "              <uri>file://media/materials/scripts/gazebo.material</uri>\n"
        "              <name>Gazebo/Grey</name>\n"
        "            </script>\n"
        "          </material>\n"
        "        </visual>\n"
        "        <velocity_decay>\n"
        "          <linear>0</linear>\n"
        "          <angular>0</angular>\n"
        "        </velocity_decay>\n"
        "        <self_collide>0</self_collide>\n"
        "        <kinematic>0</kinematic>\n"
        "        <gravity>1</gravity>\n"
        "      </link>\n"
        "      <static>0</static>\n";

TEST(SDFModelTests, test_SDFModel_Constructor){
    using boost::property_tree::ptree;
    using namespace meister;
    std::stringstream ss;
    ss << BOX_MODEL;
    ptree model_tree;

    read_xml(ss, model_tree, boost::property_tree::xml_parser::trim_whitespace);

    std::shared_ptr<SDFModel> model;

    ASSERT_NO_THROW(model.reset(new SDFModel(model_tree)));
    EXPECT_EQ(5.0 , model->center_x);
    EXPECT_EQ(6.0, model->center_y);
    EXPECT_EQ(7.0, model->center_z);
    EXPECT_EQ(1.0, model->length);
    EXPECT_EQ(2.0, model->width);
    EXPECT_EQ(3.0, model->height);


}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}