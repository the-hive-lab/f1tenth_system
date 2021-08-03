# F1Tenth Core

This repository maintains a ROS2-based core system for starting up an F1Tenth vehicle.
We built this code from the [`f1tenth_system`](https://github.com/f1tenth/f1tenth_system) repository, but stripped out all local dependencies, e.g. `joy`, `vesc`, to better use ROS2 built-in repositories.

# Getting started

1. Create a ROS workspace:
   ```shell
   $ mkdir -p f1tenth_ws/src
   $ cd f1tenth_ws/src
   ```

2. Clone the repository into the `src` directory:
   ```shell
   $ git clone https://github.com/the-hive-lab/f1tenth_system.git
   $ cd ..
   ```

3. Clone the repositories containing the unreleased package dependencies:
   ```shell
   $ vcs import --input src/f1tenth_system/f1tenth_core/vesc.repos ./src
   ```

4. Build the workspace with all the packages:
   ```shell
   $ colcon build
   ```
   **Note:** If you want to build the packages separately, you can do so by doing the following command:
   ```shell
   $ colcon build --packages-up-to <name-of-pkg>
   ```

5. Source you install directory:
   ```shell
   $ source install/setup.bash
   ```
    **Optional:** If you are installing this workspace on a fresh version of ROS, you most likely will lack all of the necessary system dependecies to run all the packages. If you are misisng missing dependecies, you can use rosdep to install them:
   ```shell
   $ rosdep install --from-paths src --ignore-src -r -y
   ```
 

6. Launch packages:
   ```shell
   $ ros2 launch f1tenth_core drive_launch.py 
   ```
This file will launch all of your nodes for the VESC, Hokuyo LiDAR, and basic `tf`. To control vehicle with a joystick, you will need to launch the`joy_teleop` package. A launch file and a custom `joy_teleop.yaml` for parameters is provided on the `f1tenth_core` package. To bring up the joystick nodes:

    ```shell
   $ ros2 launch f1tenth_core joy_launch.py
   ```
The joysticks nodes can run on the vehicle or from your workstation. Make sure you have the same `ROS_DOMAIN_ID` accross your workstation and vehicle to allow communications between nodes.
