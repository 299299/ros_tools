/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/


#pragma once

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "rosbag/player.h"
#include "ros_bag_player/JumpToNextTopic.h"

class Player
{
  public:
    Player(::rosbag::PlayerOptions const& options);
    ~Player();

    void publish();

  private:
    int readCharFromStdin();
    void setupTerminal();
    void restoreTerminal();

    void updateRateTopicTime(const ros::MessageEvent< topic_tools::ShapeShifter const >& msg_event);

    void doPublish(::rosbag::MessageInstance const& m);

    void doKeepAlive();

    void printTime();

    bool pauseCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    void processPause(const bool paused, ros::WallTime& horizon);

    void waitForSubscribers() const;

    // PLATFORM-35: new added for jump functions
    bool jumpToNextTopicCallback(ros_bag_player::JumpToNextTopic::Request& req,
                                 ros_bag_player::JumpToNextTopic::Response& res);

  private:
    typedef std::map< std::string, ros::Publisher > PublisherMap;

    ::rosbag::PlayerOptions options_;

    ros::NodeHandle node_handle_;

    ros::ServiceServer pause_service_;

    bool paused_;
    bool delayed_;

    bool pause_for_topics_;

    bool pause_change_requested_;

    bool requested_pause_state_;

    ros::Subscriber rate_control_sub_;
    ros::Time last_rate_control_;

    ros::WallTime paused_time_;

    std::vector< boost::shared_ptr< ::rosbag::Bag > > bags_;
    PublisherMap publishers_;

    // Terminal
    bool terminal_modified_;
    termios orig_flags_;
    fd_set stdin_fdset_;
    int maxfd_;

    ::rosbag::TimeTranslator time_translator_;
    ::rosbag::TimePublisher time_publisher_;

    ros::Time start_time_;
    ros::Duration bag_length_;

    // new added for jump to topic function
    ros::ServiceServer jump_topic_server_;
    std::string jump_topic_name_;
    bool jump_topic_requested_;
};