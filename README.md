# some addition tools made for ROS, based on ros1 kinectic

## Additional Features

### a new rosbag player which support jump by topics, a lot of code are copied from original rosbag player
    https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag

    while rosbag playing is paused, you can use command line to jump to the next topic by name

    ```
    rosservice call /rosbag/jump_to_next_topic your_topic_name
    ```

    meanwhile the topics between these topic frames will be also published, you can use this tool
    to achive some "debug" effect

    image if you are debuging a sensor fusion module, you can jump to time of the sensor measurement message frame by frame


### various rviz tools

    [![](https://img.youtube.com/vi/TfcgUhe6AFw/0.jpg)](https://youtu.be/TfcgUhe6AFw)




