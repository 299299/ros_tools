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