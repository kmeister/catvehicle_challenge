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

Description: contains the class definition for a ROS Node which publishes polygons
 to the /detections topic based on the point clouds published to /meister/transformed_lidar_points  and /meister/laser_points

 */
#ifndef MEISTER_OBSTACLEDETECTOR_H
#define MEISTER_OBSTACLEDETECTOR_H

#include "DetectionGrid.h"
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include "NodeController.h"
#include "RosCommunications.h"
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


using namespace std;

/** node controller that produces /detections messages from point cloud data
 *
 * controller also monitors the state of the catvehicle, and when it comes to a stop writes a worldfile representing the
 * objects detected prior to that point in time.
 */
class ObstacleDetector : public NodeController {
public:

    /** Default Constructor
     *
     */
    ObstacleDetector();

    /**Destructor
     *
     */
    ~ObstacleDetector();

    /** Publish a worldfile containing bounding volumes for all of the points recorded throughout the run
     *
     */
    void create_world();

    /** Publish a worldfile containing bounding volumes for all of the points recorded throughout the run
     * this is done on a separate thread to avoid causing an interruption to obstacle detection
     */
    void create_world_async();

    /** get a vector of Polygons for the detections topic. Note that the header for each detection will need to be set
     *
     * @param points vector of points with which to populate the detection grid.
     * @return vector of polygons representing the detected objects
     */
    vector<geometry_msgs::PolygonStamped> detect_obstacles(const vector<geometry_msgs::Point32> &points);


    /** function called periodically by run()
     *
     */
    void update() override;

    /** load parameters for the template world file, and the output filename
     *
     */
    void load_params();

    /** get this nodes name
     *
     */
    std::string controller_name() override;



private:

    ros::NodeHandle handle; //!< the Node Controller's handle, used for setting up the RosCommunicator, loading params, etc...
    RosListener<sensor_msgs::PointCloud> lidar_listener; //!< listener for for receiving /meister/transformed_lidar_points
    RosListener<sensor_msgs::PointCloud> laser_listener; //!< second listener for receiving /meister/laser_points
    RosListener<std_msgs::String> status_listener; //!< listen for vehicle status ("in_motion" or "stopped")

    int status_count; //!< status message count
    int lidar_count; //!< lidar message count
    int laser_count; //!< laser message count

    ros::Publisher detections_publisher; //!< publisher for publishing /dtections

    DetectionGrid grid_; //!< used to extract the detections from the point cloud data

    vector<Point32> all_points_; //!< stores all points received during a run to generate a worldfile at the end

    std::string world_template_filename_; //!< template file to use to create the world file
    std::string world_output_filename_; //!< filename to write a new worldfile to
    bool write_world_file_; //!< should write world file

    std::shared_ptr<boost::thread> world_creation_thread_; //!< thread for asynchronous world building
    boost::mutex all_points_mutex_; //!< mutex for locking access to all points when world creation starts
};


#endif //MEISTER_OBSTACLEDETECTOR_H
