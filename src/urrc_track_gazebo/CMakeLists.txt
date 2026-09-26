cmake_minimum_required(VERSION 3.16)
project(urrc_track_gazebo NONE)
find_package(ament_cmake QUIET)
install(DIRECTORY worlds models tracks launch docs urrc_tracks
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts
        USE_SOURCE_PERMISSIONS PATTERN "__pycache__" EXCLUDE)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
if(ament_cmake_FOUND)
  ament_package()
else()
  message(STATUS "ROS 2 is not sourced: installing standalone Gazebo resources.")
endif()
