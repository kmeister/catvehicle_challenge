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
        Contains the class definition for LidarFilter described in the README.md
*/

#ifndef MEISTER_LIDARFILTER_H
#define MEISTER_LIDARFILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include "NodeController.h"
#include "RosCommunications.h"

/** Controller for a node which filters a sensor_msgs::Pointcloud based on the distance to and height of the points
 *
 */
class LidarFilter : public NodeController{
private:
    ros::NodeHandle handle_; //!< ros communication interface for creating the communicator and loading parameters
    RosCommunicator<sensor_msgs::PointCloud, sensor_msgs::PointCloud> communicator_; //!< handles communication with other ros nodes
    tf::TransformListener listener_; //!< listen for transform from /catvehicle/velodyne_link to /catvehicle/odom

    int input_count_; //!< messages received
    double min_height_; //!< height below which the points should be rejected
    double min_distance_; //!< distance inside of which point should be rejected
    double max_distance_; //!< disatance outside of which point should be rejected

    static const double DEFAULT_MIN_HEIGHT_; //!< default height threshold
    static const double DEFAULT_MIN_DISTANCE_; //!< default min distance threshold
    static const double DEFAULT_MAX_DISTANCE_; //!< default max distance threshold

public:
    /** default constructor
     *
     */
    LidarFilter();

    /** Node controller update method
     *
     */
    void update() override;

    /** loads parameters for height and distance filters
     *
     */
    void load_params();

    /** filters points based on height and distance
     *
     * @param input point cloud to filter
     * @return PointCloud containing filtered points with the same header and channels
     */
    sensor_msgs::PointCloud  filter_points(const sensor_msgs::PointCloud &input);

    /** distance in the X/Y plane to a point
     *
     * @param point point to calculate the distance to
     * @return distance to point
     */
    double  point_distance(const geometry_msgs::Point32 &point);

    /** get min_height_ for filtered nodes
     *
     * @return min height
     */
    const double &get_min_height();

    /** get min_distance_
     *
     * @return const reference to min_distance_
     */
    const double &get_min_distance();

    /** get max_distance_
     *
     * @return const reference to max_distance_
     */
    const double &get_max_distance();

    /** get this nodes name
     *
     */
     std::string controller_name() override;
};


#endif //MEISTER_LIDARFILTER_H
