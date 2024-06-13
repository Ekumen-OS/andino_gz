cmake_minimum_required(VERSION 3.7)
project(andino_gz)

find_package(gazebo QUIET)

find_package(ament_cmake REQUIRED)

ament_python_install_package("andino_gz")

install(
  DIRECTORY
    config
    config_gui
    models
    launch
    maps
    rviz
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
