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

Description: contains constants, strings, and utility functions common to multiple classes in the meister
package.

*/

#ifndef MEISTER_COMMON_H
#define MEISTER_COMMON_H

#include <string>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

namespace meister {
    namespace CommonStrings {
        const std::string STATE_UPDATE_OUTPUT_TOPIC = "/meister/transformed_lidar_points";
        const std::string LIDAR_FILTER_OUTPUT_TOPIC = "/meister/filtered_lidar_points";
        const std::string CATVEHICLE_LIDAR_TOPIC = "/catvehicle/lidar_points";
        const std::string LASER_DATA_OUTPUT_TOPIC = "/meister/laser_points";
        const std::string VEHICLE_STATUS_OUTPUT_TOPIC = "/meister/vehicle_status";
        const std::string DETECTION_TOPIC = "/detections";
        const std::string LIDAR_COORDINATE_FRAME = "/catvehicle/velodyne_link";
        const std::string CATVEHICLE_ROOT_COORDINATE_FRAME = "/catvehicle/odom";
        const std::string OUTPUT_WORLD_FILENAME_PARAMETER = "/obstacle_detector/world_output_filename";
        const std::string CATVEHICLE_LASER_SCAN_TOPIC = "/catvehicle/front_laser_points";
        const std::string SYSTEM_VERIFICATION_BAG = "src/meister/bagfiles/system_verification.bag";

        //static const std::string LIDAR_POINTS;
    };

    namespace CommonValues {
        const double DEFAULT_RUN_RATE_ = 30.0;
    }

    namespace TestUtilities {

         /** Utility function for reading bagfiles
         *
         * @tparam MessageType the message type to read from the bagfile
         * @param topic the topic to read from the bagfile
         * @param bag an open bagfile containing a topic conforming to the above type
         * @return a vector of MessageType messages
         */
        template <typename MessageType>
        std::vector<MessageType> read_all_messages_from_topic(const std::string &topic, const rosbag::Bag &bag){
            std::vector<MessageType> all_messages;
            std::vector<std::string> topics;
            topics.push_back(topic);
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            // get all messages of MessageType published to topic
            for(rosbag::MessageInstance const m: view){
                boost::shared_ptr<MessageType> msg = m.instantiate<MessageType>();
                if (msg != NULL)
                    all_messages.push_back(*msg);
            }

            return all_messages;
        }

        /** Utility function for reading bagfiles
        *
        * @tparam MessageType the message type to read from the bagfile
        * @param topic the topic to read from the bagfile
        * @param bag an open bagfile containing a topic conforming to the above type
        * @param max_time the latest time to read messages from
        * @return a vector of MessageType messages
        */

        template <typename MessageType>
        std::vector<MessageType> read_all_messages_from_topic(const std::string &topic, const rosbag::Bag &bag, ros::Time max_time){
            std::vector<MessageType> all_messages;
            std::vector<std::string> topics;
            topics.push_back(topic);

            rosbag::View view(bag, rosbag::TopicQuery(topics), ros::TIME_MIN, max_time, false);

            // get all messages of MessageType published to topic
            for(rosbag::MessageInstance const m: view){
                boost::shared_ptr<MessageType> msg = m.instantiate<MessageType>();
                if (msg != NULL)
                    all_messages.push_back(*msg);
            }

            return all_messages;
        }

        inline std::vector<rosbag::MessageInstance> read_all_message_instances_from_topic(const std::string &topic, const rosbag::Bag &bag)
        {
            std::vector<rosbag::MessageInstance> messages;
            std::vector<std::string> topics;
            topics.push_back(topic);
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            for (auto m : view){
                messages.push_back(m);
            }

            return messages;

        }

        /** verify that a point lies withing a rectangular stamped polygon
         *
         * @param p point
         * @param ps polygon
         * @return true if p is within polygon, false if not
         */
        bool point_in(const geometry_msgs::Point32 &p, const geometry_msgs::PolygonStamped &ps){
            // assumes rectangular polygon
            if (ps.polygon.points.size() != 4){
                return false;
            }

            auto min_x = std::min(ps.polygon.points[0].x, ps.polygon.points[2].x);
            auto min_y = std::min(ps.polygon.points[0].y, ps.polygon.points[2].y);
            auto max_x = std::max(ps.polygon.points[0].x, ps.polygon.points[2].x);
            auto max_y = std::max(ps.polygon.points[0].y, ps.polygon.points[2].y);

            if (p.x < min_x || p.x > max_x){
                return false;
            }

            if (p.y < min_y || p.y > max_y){
                return false;
            }

            return true;

        }
    }


};


#endif //MEISTER_COMMON_H
