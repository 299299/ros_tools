#include "player.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"

#include <sys/select.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "rosgraph_msgs/Clock.h"

#define foreach BOOST_FOREACH

using std::map;
using std::pair;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Exception;

Player::Player(::rosbag::PlayerOptions const& options)
    : options_(options),
      paused_(false),
      // If we were given a list of topics to pause on, then go into that mode
      // by default (it can be toggled later via 't' from the keyboard).
      pause_for_topics_(options_.pause_topics.size() > 0),
      pause_change_requested_(false),
      requested_pause_state_(false),
      terminal_modified_(false),
      // PLATFORM-35: new added for jump functions
      jump_topic_requested_(false)
{
    ros::NodeHandle private_node_handle("~");
    pause_service_ = private_node_handle.advertiseService("/rosbag/pause_playback", &Player::pauseCallback, this);
    // PLATFORM-35: new added for jump to topic function
    jump_topic_server_ =
        private_node_handle.advertiseService("/rosbag/jump_to_next_topic", &Player::jumpToNextTopicCallback, this);
}

Player::~Player()
{
    foreach (shared_ptr< ::rosbag::Bag > bag, bags_)
        bag->close();

    restoreTerminal();
}

void Player::publish()
{
    options_.check();

    // Open all the bag files
    foreach (string const& filename, options_.bags)
    {
        ROS_INFO("Opening %s", filename.c_str());

        try
        {
            shared_ptr< ::rosbag::Bag > bag(boost::make_shared< ::rosbag::Bag >());
            bag->open(filename, ::rosbag::bagmode::Read);
            bags_.push_back(bag);
        }
        catch (::rosbag::BagUnindexedException ex)
        {
            std::cerr << "Bag file " << filename << " is unindexed.  Run rosbag reindex." << std::endl;
            return;
        }
    }

    setupTerminal();

    if (!node_handle_.ok())
        return;

    if (!options_.prefix.empty())
    {
        ROS_INFO_STREAM("Using prefix '" << options_.prefix << "'' for topics ");
    }

    if (!options_.quiet)
        puts("");

    // Publish all messages in the bags
    ::rosbag::View full_view;
    foreach (shared_ptr< ::rosbag::Bag > bag, bags_)
        full_view.addQuery(*bag);

    ros::Time initial_time = full_view.getBeginTime();

    initial_time += ros::Duration(options_.time);

    ros::Time finish_time = ros::TIME_MAX;
    if (options_.has_duration)
    {
        finish_time = initial_time + ros::Duration(options_.duration);
    }

    ::rosbag::View view;
    ::rosbag::TopicQuery topics(options_.topics);

    if (options_.topics.empty())
    {
        foreach (shared_ptr< ::rosbag::Bag > bag, bags_)
            view.addQuery(*bag, initial_time, finish_time);
    }
    else
    {
        foreach (shared_ptr< ::rosbag::Bag > bag, bags_)
            view.addQuery(*bag, topics, initial_time, finish_time);
    }

    if (view.size() == 0)
    {
        std::cerr << "No messages to play on specified topics.  Exiting." << std::endl;
        ros::shutdown();
        return;
    }

    // Advertise all of our messages
    foreach (const ::rosbag::ConnectionInfo* c, view.getConnections())
    {
        ros::M_string::const_iterator header_iter = c->header->find("callerid");
        std::string callerid = (header_iter != c->header->end() ? header_iter->second : string(""));

        string callerid_topic = callerid + c->topic;

        map< string, ros::Publisher >::iterator pub_iter = publishers_.find(callerid_topic);
        if (pub_iter == publishers_.end())
        {

            ros::AdvertiseOptions opts = createAdvertiseOptions(c, options_.queue_size, options_.prefix);

            ros::Publisher pub = node_handle_.advertise(opts);
            publishers_.insert(publishers_.begin(), pair< string, ros::Publisher >(callerid_topic, pub));

            pub_iter = publishers_.find(callerid_topic);
        }
    }

    if (options_.rate_control_topic != "")
    {
        std::cout << "Creating rate control topic subscriber..." << std::flush;

        boost::shared_ptr< ros::Subscriber > sub(boost::make_shared< ros::Subscriber >());
        ros::SubscribeOptions ops;
        ops.topic = options_.rate_control_topic;
        ops.queue_size = 10;
        ops.md5sum = ros::message_traits::md5sum< topic_tools::ShapeShifter >();
        ops.datatype = ros::message_traits::datatype< topic_tools::ShapeShifter >();
        ops.helper = boost::make_shared<
            ros::SubscriptionCallbackHelperT< const ros::MessageEvent< topic_tools::ShapeShifter const >& > >(
            boost::bind(&Player::updateRateTopicTime, this, _1));

        rate_control_sub_ = node_handle_.subscribe(ops);

        std::cout << " done." << std::endl;
    }

    std::cout << "Waiting " << options_.advertise_sleep.toSec() << " seconds after advertising topics..." << std::flush;
    options_.advertise_sleep.sleep();
    std::cout << " done." << std::endl;

    std::cout << std::endl << "Hit space to toggle paused, or 's' to step." << std::endl;

    paused_ = options_.start_paused;

    if (options_.wait_for_subscribers)
    {
        waitForSubscribers();
    }

    while (true)
    {
        // Set up our time_translator and publishers

        time_translator_.setTimeScale(options_.time_scale);

        start_time_ = view.begin()->getTime();
        time_translator_.setRealStartTime(start_time_);
        bag_length_ = view.getEndTime() - view.getBeginTime();

        // Set the last rate control to now, so the program doesn't start delayed.
        last_rate_control_ = start_time_;

        time_publisher_.setTime(start_time_);

        ros::WallTime now_wt = ros::WallTime::now();
        time_translator_.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));

        time_publisher_.setTimeScale(options_.time_scale);
        if (options_.bag_time)
            time_publisher_.setPublishFrequency(options_.bag_time_frequency);
        else
            time_publisher_.setPublishFrequency(-1.0);

        paused_time_ = now_wt;

        // Call do-publish for each message
        foreach (::rosbag::MessageInstance m, view)
        {
            if (!node_handle_.ok())
                break;
            doPublish(m);
        }

        if (options_.keep_alive)
            while (node_handle_.ok())
                doKeepAlive();

        if (!node_handle_.ok())
        {
            std::cout << std::endl;
            break;
        }
        if (!options_.loop)
        {
            std::cout << std::endl << "Done." << std::endl;
            break;
        }
    }

    ros::shutdown();
}

void Player::updateRateTopicTime(const ros::MessageEvent< topic_tools::ShapeShifter const >& msg_event)
{
    boost::shared_ptr< topic_tools::ShapeShifter const > const& ssmsg = msg_event.getConstMessage();
    std::string def = ssmsg->getMessageDefinition();
    size_t length = ros::serialization::serializationLength(*ssmsg);

    // Check the message definition.
    std::istringstream f(def);
    std::string s;
    bool flag = false;
    while (std::getline(f, s, '\n'))
    {
        if (!s.empty() && s.find("#") != 0)
        {
            // Does not start with #, is not a comment.
            if (s.find("Header ") == 0)
            {
                flag = true;
            }
            break;
        }
    }
    // If the header is not the first element in the message according to the definition, throw an error.
    if (!flag)
    {
        std::cout << std::endl
                  << "WARNING: Rate control topic is bad, header is not first. MSG may be malformed." << std::endl;
        return;
    }

    std::vector< uint8_t > buffer(length);
    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer< topic_tools::ShapeShifter >::write(ostream, *ssmsg);

    // Assuming that the header is the first several bytes of the message.
    // uint32_t header_sequence_id   = buffer[0] | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2] << 16 |
    // (uint32_t)buffer[3] << 24;
    int32_t header_timestamp_sec =
        buffer[4] | (uint32_t)buffer[5] << 8 | (uint32_t)buffer[6] << 16 | (uint32_t)buffer[7] << 24;
    int32_t header_timestamp_nsec =
        buffer[8] | (uint32_t)buffer[9] << 8 | (uint32_t)buffer[10] << 16 | (uint32_t)buffer[11] << 24;

    last_rate_control_ = ros::Time(header_timestamp_sec, header_timestamp_nsec);
}

void Player::printTime()
{
    if (!options_.quiet)
    {

        ros::Time current_time = time_publisher_.getTime();
        ros::Duration d = current_time - start_time_;

        if (paused_)
        {
            printf("\r [PAUSED ]  Bag Time: %13.6f   Duration: %.6f / %.6f               \r",
                   time_publisher_.getTime().toSec(),
                   d.toSec(),
                   bag_length_.toSec());
        }
        else if (delayed_)
        {
            ros::Duration time_since_rate = std::max(ros::Time::now() - last_rate_control_, ros::Duration(0));
            printf("\r [DELAYED]  Bag Time: %13.6f   Duration: %.6f / %.6f   Delay: %.2f \r",
                   time_publisher_.getTime().toSec(),
                   d.toSec(),
                   bag_length_.toSec(),
                   time_since_rate.toSec());
        }
        else
        {
            printf("\r [RUNNING]  Bag Time: %13.6f   Duration: %.6f / %.6f               \r",
                   time_publisher_.getTime().toSec(),
                   d.toSec(),
                   bag_length_.toSec());
        }
        fflush(stdout);
    }
}

bool Player::pauseCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    pause_change_requested_ = (req.data != paused_);
    requested_pause_state_ = req.data;

    res.success = pause_change_requested_;

    if (res.success)
    {
        res.message = std::string("Playback is now ") + (requested_pause_state_ ? "paused" : "resumed");
    }
    else
    {
        res.message = std::string("Bag is already ") + (requested_pause_state_ ? "paused." : "running.");
    }

    return true;
}

void Player::processPause(const bool paused, ros::WallTime& horizon)
{
    paused_ = paused;

    if (paused_)
    {
        paused_time_ = ros::WallTime::now();
    }
    else
    {
        // Make sure time doesn't shift after leaving pause.
        ros::WallDuration shift = ros::WallTime::now() - paused_time_;
        paused_time_ = ros::WallTime::now();

        time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

        horizon += shift;
        time_publisher_.setWCHorizon(horizon);
    }
}

void Player::waitForSubscribers() const
{
    bool all_topics_subscribed = false;
    std::cout << "Waiting for subscribers." << std::endl;
    while (!all_topics_subscribed)
    {
        all_topics_subscribed = true;
        foreach (const PublisherMap::value_type& pub, publishers_)
        {
            all_topics_subscribed &= pub.second.getNumSubscribers() > 0;
        }
        ros::Duration(0.1).sleep();
    }
    std::cout << "Finished waiting for subscribers." << std::endl;
}

void Player::doPublish(::rosbag::MessageInstance const& m)
{

    string const& topic = m.getTopic();
    ros::Time const& time = m.getTime();
    string callerid = m.getCallerId();

    ros::Time translated = time_translator_.translate(time);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(time);
    time_publisher_.setWCHorizon(horizon);

    string callerid_topic = callerid + topic;

    map< string, ros::Publisher >::iterator pub_iter = publishers_.find(callerid_topic);
    ROS_ASSERT(pub_iter != publishers_.end());

    // Update subscribers.
    ros::spinOnce();

    // If immediate specified, play immediately
    if (options_.at_once)
    {
        time_publisher_.stepClock();
        pub_iter->second.publish(m);
        printTime();
        return;
    }

    // If skip_empty is specified, skip this region and shift.
    if (time - time_publisher_.getTime() > options_.skip_empty)
    {
        time_publisher_.stepClock();

        ros::WallDuration shift = ros::WallTime::now() - horizon;
        time_translator_.shift(ros::Duration(shift.sec, shift.nsec));
        horizon += shift;
        time_publisher_.setWCHorizon(horizon);
        (pub_iter->second).publish(m);

        return;
    }

    if (pause_for_topics_)
    {
        for (std::vector< std::string >::iterator i = options_.pause_topics.begin(); i != options_.pause_topics.end();
             ++i)
        {
            if (topic == *i)
            {
                paused_ = true;
                paused_time_ = ros::WallTime::now();
            }
        }
    }

    // Check if the rate control topic has posted recently enough to continue, or if a delay is needed.
    // Delayed is separated from paused to allow more verbose printing.
    if (rate_control_sub_ != NULL)
    {
        if ((time_publisher_.getTime() - last_rate_control_).toSec() > options_.rate_control_max_delay)
        {
            delayed_ = true;
            paused_time_ = ros::WallTime::now();
        }
    }

    while ((paused_ || delayed_ || !time_publisher_.horizonReached()) && node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_.ok())
        {
            ros::spinOnce();

            if (pause_change_requested_)
            {
                processPause(requested_pause_state_, horizon);
                pause_change_requested_ = false;
            }

            switch (readCharFromStdin())
            {
                case ' ':
                    processPause(!paused_, horizon);
                    break;
                case 's':
                    if (paused_)
                    {
                        time_publisher_.stepClock();

                        ros::WallDuration shift = ros::WallTime::now() - horizon;
                        paused_time_ = ros::WallTime::now();

                        time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                        horizon += shift;
                        time_publisher_.setWCHorizon(horizon);

                        (pub_iter->second).publish(m);

                        printTime();
                        return;
                    }
                    break;
                case 't':
                    pause_for_topics_ = !pause_for_topics_;
                    break;
                case EOF:
                    if (paused_)
                    {
                        printTime();
                        time_publisher_.runStalledClock(ros::WallDuration(.1));
                        ros::spinOnce();
                    }
                    else if (delayed_)
                    {
                        printTime();
                        time_publisher_.runStalledClock(ros::WallDuration(.1));
                        ros::spinOnce();
                        // You need to check the rate here too.
                        if (rate_control_sub_ == NULL ||
                            (time_publisher_.getTime() - last_rate_control_).toSec() <= options_.rate_control_max_delay)
                        {
                            delayed_ = false;
                            // Make sure time doesn't shift after leaving delay.
                            ros::WallDuration shift = ros::WallTime::now() - paused_time_;
                            paused_time_ = ros::WallTime::now();

                            time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                            horizon += shift;
                            time_publisher_.setWCHorizon(horizon);
                        }
                    }
                    else
                        charsleftorpaused = false;
            }
        }

        printTime();
        time_publisher_.runClock(ros::WallDuration(.1));
        ros::spinOnce();
    }

    pub_iter->second.publish(m);

    // PLATFORM-35: new added for jump to topic function
    // we should make sure topic is sent out then we can pause here
    if (jump_topic_requested_)
    {
        if (topic.find(jump_topic_name_) != std::string::npos)
        {
            paused_ = true;
            paused_time_ = ros::WallTime::now();
            jump_topic_requested_ = false;
            printTime();
        }
    }
}

void Player::doKeepAlive()
{
    // Keep pushing ourself out in 10-sec increments (avoids fancy math dealing with the end of time)
    ros::Time const& time = time_publisher_.getTime() + ros::Duration(10.0);

    ros::Time translated = time_translator_.translate(time);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(time);
    time_publisher_.setWCHorizon(horizon);

    if (options_.at_once)
    {
        return;
    }

    // If we're done and just staying alive, don't watch the rate control topic. We aren't publishing anyway.
    delayed_ = false;

    while ((paused_ || !time_publisher_.horizonReached()) && node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_.ok())
        {
            switch (readCharFromStdin())
            {
                case ' ':
                    paused_ = !paused_;
                    if (paused_)
                    {
                        paused_time_ = ros::WallTime::now();
                    }
                    else
                    {
                        // Make sure time doesn't shift after leaving pause.
                        ros::WallDuration shift = ros::WallTime::now() - paused_time_;
                        paused_time_ = ros::WallTime::now();

                        time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                        horizon += shift;
                        time_publisher_.setWCHorizon(horizon);
                    }
                    break;
                case EOF:
                    if (paused_)
                    {
                        printTime();
                        time_publisher_.runStalledClock(ros::WallDuration(.1));
                        ros::spinOnce();
                    }
                    else
                        charsleftorpaused = false;
            }
        }

        printTime();
        time_publisher_.runClock(ros::WallDuration(.1));
        ros::spinOnce();
    }
}

void Player::setupTerminal()
{
    if (terminal_modified_)
        return;

    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
    flags.c_cc[VMIN] = 0;      // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;     // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);

    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
    terminal_modified_ = true;
}

void Player::restoreTerminal()
{
    if (!terminal_modified_)
        return;
    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);
    terminal_modified_ = false;
}

int Player::readCharFromStdin()
{

    fd_set testfd = stdin_fdset_;
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
        return EOF;
    return getc(stdin);
}

// new added for jump to topic function
bool Player::jumpToNextTopicCallback(ros_bag_player::JumpToNextTopic::Request& req,
                                     ros_bag_player::JumpToNextTopic::Response&)
{
    jump_topic_requested_ = true;
    jump_topic_name_ = req.topic_name.data;
    // if you want to jump to next frame, you should at least make bag playing not pause
    pause_change_requested_ = true;
    requested_pause_state_ = false;
    return true;
}