cmake_minimum_required(VERSION 3.7)
project(andino_gz)

find_package(gazebo QUIET)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
