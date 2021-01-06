#include "boost/program_options.hpp"
#include "player.h"

namespace po = boost::program_options;

rosbag::PlayerOptions parseOptions(int argc, char** argv)
{
    rosbag::PlayerOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()("help,h", "produce help message")(
        "prefix,p", po::value< std::string >()->default_value(""), "prefixes all output topics in replay")(
        "quiet,q", "suppress console output")("immediate,i", "play back all messages without waiting")(
        "pause", "start in paused mode")("queue",
                                         po::value< int >()->default_value(100),
                                         "use an outgoing queue of size SIZE")("clock", "publish the clock time")(
        "hz", po::value< float >()->default_value(100.0f), "use a frequency of HZ when publishing clock time")(
        "delay,d",
        po::value< float >()->default_value(0.2f),
        "sleep SEC seconds after every advertise call (to allow subscribers to connect)")(
        "rate,r", po::value< float >()->default_value(1.0f), "multiply the publish rate by FACTOR")(
        "start,s", po::value< float >()->default_value(0.0f), "start SEC seconds into the bag files")(
        "duration,u", po::value< float >(), "play only SEC seconds from the bag files")(
        "skip-empty", po::value< float >(), "skip regions in the bag with no messages for more than SEC seconds")(
        "loop,l", "loop playback")("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")(
        "try-future-version", "still try to open a bag file, even if the version is not known to the player")(
        "topics", po::value< std::vector< std::string > >()->multitoken(), "topics to play back")(
        "pause-topics", po::value< std::vector< std::string > >()->multitoken(), "topics to pause playback on")(
        "bags", po::value< std::vector< std::string > >(), "bag files to play back from")(
        "wait-for-subscribers", "wait for at least one subscriber on each topic before publishing")(
        "rate-control-topic",
        po::value< std::string >(),
        "watch the given topic, and if the last publish was more than <rate-control-max-delay> ago, wait until the "
        "topic publishes again to continue playback")(
        "rate-control-max-delay",
        po::value< float >()->default_value(1.0f),
        "maximum time difference from <rate-control-topic> before pausing");

    po::positional_options_description p;
    p.add("bags", -1);

    po::variables_map vm;

    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    }
    catch (boost::program_options::invalid_command_line_syntax& e)
    {
        throw ros::Exception(e.what());
    }
    catch (boost::program_options::unknown_option& e)
    {
        throw ros::Exception(e.what());
    }

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    if (vm.count("prefix"))
        opts.prefix = vm["prefix"].as< std::string >();
    if (vm.count("quiet"))
        opts.quiet = true;
    if (vm.count("immediate"))
        opts.at_once = true;
    if (vm.count("pause"))
        opts.start_paused = true;
    if (vm.count("queue"))
        opts.queue_size = vm["queue"].as< int >();
    if (vm.count("hz"))
        opts.bag_time_frequency = vm["hz"].as< float >();
    if (vm.count("clock"))
        opts.bag_time = true;
    if (vm.count("delay"))
        opts.advertise_sleep = ros::WallDuration(vm["delay"].as< float >());
    if (vm.count("rate"))
        opts.time_scale = vm["rate"].as< float >();
    if (vm.count("start"))
    {
        opts.time = vm["start"].as< float >();
        opts.has_time = true;
    }
    if (vm.count("duration"))
    {
        opts.duration = vm["duration"].as< float >();
        opts.has_duration = true;
    }
    if (vm.count("skip-empty"))
        opts.skip_empty = ros::Duration(vm["skip-empty"].as< float >());
    if (vm.count("loop"))
        opts.loop = true;
    if (vm.count("keep-alive"))
        opts.keep_alive = true;
    if (vm.count("wait-for-subscribers"))
        opts.wait_for_subscribers = true;

    if (vm.count("topics"))
    {
        std::vector< std::string > topics = vm["topics"].as< std::vector< std::string > >();
        for (std::vector< std::string >::iterator i = topics.begin(); i != topics.end(); i++)
            opts.topics.push_back(*i);
    }

    if (vm.count("pause-topics"))
    {
        std::vector< std::string > pause_topics = vm["pause-topics"].as< std::vector< std::string > >();
        for (std::vector< std::string >::iterator i = pause_topics.begin(); i != pause_topics.end(); i++)
            opts.pause_topics.push_back(*i);
    }

    if (vm.count("rate-control-topic"))
        opts.rate_control_topic = vm["rate-control-topic"].as< std::string >();

    if (vm.count("rate-control-max-delay"))
        opts.rate_control_max_delay = vm["rate-control-max-delay"].as< float >();

    if (vm.count("bags"))
    {
        std::vector< std::string > bags = vm["bags"].as< std::vector< std::string > >();
        for (std::vector< std::string >::iterator i = bags.begin(); i != bags.end(); i++)
            opts.bags.push_back(*i);
    }
    else
    {
        if (vm.count("topics") || vm.count("pause-topics"))
            throw ros::Exception(
                "When using --topics or --pause-topics, --bags "
                "should be specified to list bags.");
        throw ros::Exception("You must specify at least one bag to play back.");
    }

    return opts;
}

int main(int argc, char** argv)
{
    const char* ascii_logo =
        "     ______     ____        __  ____                ____  _\n\
    / ___\\ \\   / /\\ \\      / / | __ )  __ _  __ _  |  _ \\| | __ _ _   _  ___ _ __\n\
    \\___  \\ \\ / /  \\ \\ /\\ / /  |  _ \\ / _` |/ _` | | |_) | |/ _` | | | |/ _ \\ '__|\n\
     ___) |\\ V /    \\ V  V /   | |_) | (_| | (_| | |  __/| | (_| | |_| |  __/ |\n\
    |____/  \\_/      \\_/\\_/    |____/ \\__,_|\\__, | |_|   |_|\\__,_|\\__, |\\___|_|\n\
                                            |___/                 |___/\n";
    printf("%s\n", ascii_logo);

    ros::init(argc, argv, "play", ros::init_options::AnonymousName);

    // Parse the command-line options
    rosbag::PlayerOptions opts;
    try
    {
        opts = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex)
    {
        ROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }

    Player player(opts);

    try
    {
        player.publish();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL("%s", e.what());
        return 1;
    }

    return 0;
}