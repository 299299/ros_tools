cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ros_bag_player)

set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wall -Wextra -Werror")

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             message_generation
             std_msgs
             rosbag)

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# Generate services in the 'srv' folder
add_service_files(FILES JumpToNextTopic.srv)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

set(SRC_FILES ${PROJECT_SOURCE_DIR}/src/main.cpp
              ${PROJECT_SOURCE_DIR}/src/player.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
