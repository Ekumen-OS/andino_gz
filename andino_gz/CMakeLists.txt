cmake_minimum_required(VERSION 3.7)
project(andino_gz)

find_package(gazebo QUIET)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    models
    launch
    rviz
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
