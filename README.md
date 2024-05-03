# Andino Gazebo (Fortress) Simulation


<img src="./docs/media/andino_gz.png" width="800"/>

## :clipboard: Description

This package provides a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in Gazebo Fortress using [ros_gz](https://github.com/gazebosim/ros_gz) to integrate it with ROS 2.

## :clamp: Platforms

- ROS 2: Humble Hawksbill
- OS:
  - Ubuntu 22.04 Jammy Jellyfish
- Gazebo:
  - Fortress

## :inbox_tray: Installation

1. Clone this repository

```sh
git clone git@github.com:ekumenlabs/andino_gz.git
```

2. Set up docker environment:
Refer to [docker readme](docker/README.md)

Once the container is running and dependencies have been installed you can proceed to package building.

## :package: Build

The package contains some dependencies that must be installed in order to build it:

```
rosdep install --from-paths src -i -y
```

Then build the package and source the install workspace. To do so run the following commands:

```sh
colcon build
source install/setup.bash
```

## :rocket: Usage

### Andino simulation

Once the package is built and sourced, you can start a simulation.

  ```sh
  ros2 launch andino_gz andino_gz.launch.py
  ```

_Note: You can use `--world_name` flag to indicate other [world](andino_gz/worlds/) to use. (For example: `depot.sdf`(default), `empty.sdf`)_

By default the ros bridge and rviz is initialized. In case you prefer to disable any of those you can do it via its flags:

  ```sh
  ros2 launch andino_gz andino_gz.launch.py ros_bridge:=false rviz:=false
  ```

To see a complete list of available arguments for the launch file do:
  ```sh
  ros2 launch andino_gz andino_gz.launch.py --show-args
  ```

Make sure to review the required topics using `ign topics` and `ros2 topic` CLI tools.
Also, consider using looking at the translation entries under `andino_gz/config/bridge_config.yaml`.

### Multi robot simulation

  This simulation also support multi robot simulation.

  ```sh
  ros2 launch andino_gz andino_gz.launch.py robots:="
       andino1={x: 0.0, y: 0.0, z: 0.1, yaw: 0.};
       andino2={x: -0.4, y: 0.1, z: 0.1, yaw: 0.};
       andino3={x: -0.4, y: -0.1, z: 0.1, yaw: 0.};
       andino4={x: -0.8, y: 0.2, z: 0.1, yaw: 0.};
       andino5={x: -0.8, y: -0.2, z: 0.1, yaw: 0.};
       andino6={x: -0.8, y: 0.0, z: 0.1, yaw: 0.};"
  ```

  _Note: You can add as many as you want_

  <img src="./docs/media/andino_gz_multi_robot.png" width="800"/>

  The launch file is in charge of:
   - Start Gazebo simulator with a defined world (See '--world_name' flag)
   - Spawn as many robots as commanded.
   - Launch ros bridge for each robot.
   - Launch rviz visualization for each robot.

  The simulation allows spawn as many robots as you want via the `--robots` flags.
  For that you can pass the information of the robots in some sort of YAML format via ROS2 cli:
  ```yaml
    <robot_name>={x: 0.0, y: 0.0, yaw: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0};
  ```

  Note a ROS Namespace is pushed for each robot so all the topics and nodes are called the same with a difference of a `<robot_name>` prefix.


### SLAM

<img src="./docs/media/andino_slam.png" width="800"/>

1. Run simulation with ros bridge and RViz.

    ```sh
    ros2 launch andino_gz andino_gz.launch.py
    ```

2. Run slam toolbox
    ```
    ros2 launch andino_gz slam_toolbox_online_async.launch.py
    ```

    Configuration can be forwarded to the `slam_toolbox_node`. By default, the configuration parameters are obtained from [andino's configuration file](https://github.com/Ekumen-OS/andino/blob/humble/andino_slam/config/slam_toolbox_online_async.yaml). In case a custom file is wanted to be passed, simply use the launch file argument for indicating the path to a new file.

    ```
    ros2 launch andino_gz slam_toolbox_online_async.launch.py slams_param_file:=<my_path>
    ```

3. Visualize in RViz: Add `map` panel to RViz and see how the map is being generated.

## :raised_hands: Contributing

Issues or PRs are always welcome! Please refer to [CONTRIBUTING](CONTRIBUTING.md) doc.

## Code development

Note that a [`Docker`](./docker) folder is provided for easy setting up the workspace.
