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
        defines a utility class used in the verification of the ObstacleDetector class
*/

#ifndef MEISTER_SDFMODEL_H_H
#define MEISTER_SDFMODEL_H_H

#include <sstream>
#include <boost/property_tree/xml_parser.hpp>
#include <geometry_msgs/Point32.h>
#include <iostream>

namespace meister {
    /** parses model data from a worldfile into dimension and position data.
    *
    */
    struct SDFModel {
        double length;
        double width;
        double height;
        double center_x;
        double center_y;
        double center_z;
        std::string size_string;

        SDFModel(const boost::property_tree::ptree &model_tree){
            std::stringstream ss;
            ss << model_tree.get<std::string>("pose");

            ss >> center_x >> center_y >> center_z;

           std::stringstream ss2;

            ss2 << model_tree.get<std::string>("link.collision.geometry.box.size");
            size_string = ss2.str();

            ss2 >> length;
            ss2 >> width;
            ss2 >> height;

        }

        /** test if a given geometry_msgs::Point32 is within this bounding volume
         *
         * @param point point to be tested
         * @return true if the point is within the bounding volume, false if it is not
         */
        bool in_model(const geometry_msgs::Point32 &point){
            if ( point.x < center_x - length / 2 || point.x > center_x + length / 2){
                return false;
            }

            if ( point.y < center_y - width / 2 || point.y > center_y + width / 2 ){
                return false;
            }

            if (point.z < center_z - height / 2 || point.z > center_z + height / 2){
                return false;
            }

            return true;
        }
    };
}

#endif //MEISTER_SDFMODEL_H_H
