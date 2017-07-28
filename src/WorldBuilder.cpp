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

*/

#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "../include/meister/WorldBuilder.h"
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>

using namespace std;
using boost::property_tree::ptree;

WorldBuilder::WorldBuilder() : model_count(0), model_name_root("aabb_model_"){}

ptree WorldBuilder::create_model_instance(const Dimension dim, const Pose pose, const std::string &model_name){
    ptree instance_tree;
    std::stringstream ss;
    ss << BOX_MODEL_INSTANCE_;

    read_xml(ss, instance_tree, boost::property_tree::xml_parser::trim_whitespace);

    ss.str(std::string());
    ss << pose;

    instance_tree.put("model.pose", ss.str());
    instance_tree.put("model.link.pose", ss.str());
    instance_tree.put("model.<xmlattr>.name", model_name);

    return instance_tree;

}

ptree WorldBuilder::create_model(const Dimension dim, const Pose pose, const std::string &model_name) {
    ptree model_tree;
    std::stringstream tmp_stream;

    tmp_stream << BOX_MODEL_;
    read_xml(tmp_stream, model_tree, boost::property_tree::xml_parser::trim_whitespace);

    model_tree.put("model.<xmlattr>.name", model_name);

    tmp_stream.str(std::string());
    tmp_stream << dim;

    model_tree.put("model.link.collision.geometry.box.size", tmp_stream.str());
    model_tree.put("model.link.visual.geometry.box.size", tmp_stream.str());

    tmp_stream.str(std::string()); // clear the stringstream

    tmp_stream << pose;

    model_tree.put("model.pose", tmp_stream.str());

    return model_tree;

}

void WorldBuilder::load_template(const std::string &template_file) {
    read_xml(template_file, world_, boost::property_tree::xml_parser::trim_whitespace);
}

void WorldBuilder::add_model(const ptree &model, const ptree &model_instance){



    auto &subtree2 = world_.get_child("sdf.world.state");

    subtree2.push_back(std::make_pair("model", model_instance.get_child("model")));

    world_.put_child("sdf.world.state", subtree2);

    auto &subtree = world_.get_child("sdf.world");
    subtree.push_back(std::make_pair("model", model.get_child("model")));
    world_.put_child("sdf.world", subtree);

}

void WorldBuilder::write_world(const std::string &output_filename) {
    
    boost::property_tree::xml_writer_settings<char> settings('\t', 1);
    write_xml(output_filename, world_, std::locale(), settings);
}

void WorldBuilder::insert_aabb(const geometry_msgs::PolygonStamped &poly) {
    // assume points are SW SE NE NW
    geometry_msgs::Point32 point_0(poly.polygon.points[0]);
    geometry_msgs::Point32 point_2(poly.polygon.points[2]);

    WorldBuilder::Dimension dim{point_2.x - point_0.x,
                                point_2.y - point_0.y,
                                point_0.z};

    WorldBuilder::Pose pose{(point_0.x + dim.length / 2.0),
                            (point_0.y + dim.length / 2.0),
                            (dim.height / 2.0),
                            0, 0 , 0 };

    std::stringstream ss;
    ss << model_name_root << model_count;

    ptree model = create_model(dim, pose, ss.str());
    ptree model_instance = create_model_instance(dim, pose, ss.str());

    add_model(model, model_instance);

    model_count++;
}

const std::string WorldBuilder::BOX_MODEL_INSTANCE_ = "<model name='model_name'>\n"
        "        <pose>0 0 0 0 0 0</pose>\n"
        "        <link name='link'>\n"
        "          <pose>0 0 0 0 0 0</pose>\n"
        "          <velocity>0 0 0 0 0 0</velocity>\n"
        "          <acceleration>0 0 0 0 0 0</acceleration>\n"
        "          <wrench>0 0 0 0 0 0</wrench>\n"
        "        </link>\n"
        "      </model>";

const std::string WorldBuilder::BOX_MODEL_ = "<model name='unit_box_1'>\n"
        "      <pose>0 0 0 0 0 0</pose>\n"
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
        "              <size>1 1 1</size>\n"
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
        "      <static>0</static>\n"
        "    </model>";
