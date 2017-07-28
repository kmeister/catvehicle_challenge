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
class definition for a class used in the construction of world files
*/

#ifndef MEISTER_WORLDBUILDER_H
#define MEISTER_WORLDBUILDER_H

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <geometry_msgs/PolygonStamped.h>

using boost::property_tree::ptree;

/** Class Resposnible for writing a worldfile from a set of polygons
 *
 */
class WorldBuilder {
private:

    /** Utility structure for passing around bounding volume dimensions
     *
     */
    struct Dimension {
        float length;
        float width;
        float height;

        friend std::ostream &operator<<(std::ostream &os, const Dimension &dim) {
            os << dim.length << " " << dim.width << " " << dim.height;
            return os;
        }
    };

    /** Utility Structure for passing around bounding volume poses
     *
     */
    struct Pose {
        double x;
        double y;
        double z;
        double rx;
        double ry;
        double rz;

        friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
            os << pose.x << " ";
            os << pose.y << " ";
            os << pose.z << " ";
            os << pose.rx << " ";
            os << pose.ry << " ";
            os << pose.rz;
            return os;
        }
    };

    std::string empty_world_filename_; //!< empty worldfile provided as a template to populate
    std::string output_filename_; //!< output worldfile name

    ptree world_; //!< property tree representing the world to be written

    int model_count; //!<< number of models inserted into world file;

    std::string model_name_root;

    /**insert a model into the world and world state
     *
     * @param model detailed model data
     * @param model_instance instance model data added to the world state
     *
     */
    void add_model(const ptree &model, const ptree &model_instance);

    /** Helper function to create a ptree for a Bounding Volume defined by a dimension and pose
     *
     * @param dim dimensions of the bounding volume
     * @param pose position of the bounding volume
     * @param model_name name for the model
     * @return a ptree that can be used to create the xml for the model
     */
    ptree create_model(const WorldBuilder::Dimension dim, const WorldBuilder::Pose pose, const std::string &model_name);

    /** Helper function to create a ptree  representing the XML for a Bounding Volume world state instance
     *
     * @param dim dimension of the bounding box
     * @param pose pose of the bounding box
     * @param model_name name of the model
     * @return ptree representing the model instance xml
     */
    ptree create_model_instance(const WorldBuilder::Dimension dim, const WorldBuilder::Pose pose, const std::string &model_name);



    const static std::string BOX_MODEL_; //!< xml string for a box model
    const static std::string BOX_MODEL_INSTANCE_; //!< xml string for a box model instance

public:
    /** Creates a new World Builder with model_name_root = "aabb_model_" and model_count = 0
     *
     */
    WorldBuilder();

    /** load the template file specified by empty_world_filename_
     *
     */
    void load_template(const std::string &template_file);

    /** write the worldfile
     *
     */
    void write_world(const std::string &output_filename);

    /** add a rectangular prism into the world
     *
     * @param poly a stamped polygon made of four points representing the box to be inserted the height of the box is the first point's z component
     */
    void insert_aabb(const geometry_msgs::PolygonStamped &poly);

};


#endif //MEISTER_WORLDBUILDER_H
