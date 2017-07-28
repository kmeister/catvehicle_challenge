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
        Defines some generic objects for use with Node Controllers for the purposes of ros communications
*/

#ifndef MEISTER_ROSPROXY_H
#define MEISTER_ROSPROXY_H

#include <ros/ros.h>

template <typename InputType>
class RosListener {
private:
    ros::Subscriber subscriber_;

    InputType input_; //!< stores the most recently received input message

    uint64_t input_count_; //!< counts the number of messages received by this Listener

public:
    RosListener(std::string input_topic, ros::NodeHandle &handle): input_count_(0), input_(){
        subscriber_ = handle.subscribe(input_topic, 1000, &RosListener<InputType>::setInput, this);
    };

    /** accessor for the most recently received input message
     *
     * @return const reference to the most recently received input message
     */
    const InputType &input(){
        return input_;
    }

    /** Setter for input_ used as the callback for subscriber_
     *
     * @param input value to set input_ to
     */
    void setInput(const InputType &input){
        input_ = input;
        input_count_++;
    }

    /** input count accessor
     *
     * @return number of times input has been set since this object was instantiated
     */
    uint64_t input_count(){
        return input_count_;
    }
};



/** Template class for an object that can receive one type of ROS message and Publish another
 *
 * @tparam InputType
 * @tparam OutputType
 */
template < typename InputType, typename OutputType>
class RosCommunicator{
public:


private:
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;

    InputType input_; //!< stores the most recently received input message

    uint64_t input_count_; //!< counts the number of messages received by this RosTalker

public:
    RosCommunicator(std::string input_topic, std::string output_topic, ros::NodeHandle &handle): input_count_(0){
        subscriber_ = handle.subscribe(input_topic, 1000, &RosCommunicator<InputType,OutputType>::setInput, this);
        publisher_ = handle.advertise<OutputType>(output_topic, 1000);
    };

    /** accessor for the most recently received input message
     *
     * @return const reference to the most recently received input message
     */
    const InputType &input(){
        return input_;
    }

    /** Setter for input_ used as the callback for subscriber_
     *
     * @param input value to set input_ to
     */
    void setInput(const InputType &input){
        input_ = input;
        input_count_++;
    }

    /** proxy for the publish method of the publisher
     *
     * @param message message to publish
     */
    void publish(const OutputType &message){
        publisher_.publish(message);
    }

    /** input count accessor
     *
     * @return number of times input has been set since this object was instantiated
     */
    uint64_t input_count(){
        return input_count_;
    }

};

#endif //MEISTER_ROSPROXY_H
