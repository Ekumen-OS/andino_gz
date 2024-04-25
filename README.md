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

Once the package is built and sourced, you can start a simulation.

  ```sh
  ros2 launch andino_gz andino_gz.launch.py
  ```

_Note: You can use `--world_name` flag to indicate other [world](andino_gz/worlds/) to use. (For example: `depot.sdf`(default), `empty.sdf`)_

If you'd like to work from ROS you can launch the ros bridge by adding the corresponding flag

  ```sh
  ros2 launch andino_gz andino_gz.launch.py ros_bridge:=true
  ```

(Optional) Or launching it separately via:

  ```sh
  ros2 launch andino_gz gz_ros_bridge.launch.py
  ```

Make sure to review the required topics using `ign topics` and `ros2 topic` CLI tools.
Also, consider using looking at the translation entries under `andino_gz/config/bridge_config.yaml`.

### Spawn multiple Andinos

Launch simulation as before:
  ```sh
  ros2 launch andino_gz andino_gz.launch.py
  ```

This will spawn only one Andino in the simulation

For spawning more Andinos you can use the `spawn_robot` launch file. Make sure a different `entity` name is passed as argument as well as initial positions.
  ```sh
  ros2 launch andino_gz spawn_robot.launch.py entity:=andino_n initial_pose_x:=1 initial_pose_y:=1
  ```

## :raised_hands: Contributing

Issues or PRs are always welcome! Please refer to [CONTRIBUTING](CONTRIBUTING.md) doc.

## Code development

Note that a [`Docker`](./docker) folder is provided for easy setting up the workspace.
