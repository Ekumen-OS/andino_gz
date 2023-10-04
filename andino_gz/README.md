# Description
A Gazebo simulation of the Andino Package.

To build the package, run
`colcon build`

To run the simulation, source and run
`ros2 launch andino_gz andino_gz.launch.py`

#### Launch file arguments
- 'jsp_gui':
    - Run [`joint state publisher gui`](https://github.com/ros/joint_state_publisher/tree/ros2) node. (default: 'false')
- 'rsp':
    - Run [`robot state publisher`](https://github.com/ros/robot_state_publisher) node. (default: 'false')
- 'rviz':
    - Start RViz. (default: 'false')
