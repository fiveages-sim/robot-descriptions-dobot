# Dobot CR5 Description

This package contains the description files for Dobot CR5 Manipulator. The origin models could be found
at [DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4).

## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to cr5_description --symlink-install
```

## 2. Visualize the robot

* Without gripper
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch robot_common_launch manipulator.launch.py robot:=cr5
    ```
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch robot_common_launch manipulator.launch.py robot:=cr5 collider:=simple
    ```
  ![cr5](../../.images/dobot_cr5.png)

* With Robotiq 85 Gripper
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch robot_common_launch manipulator.launch.py robot:=cr5 type:="robotiq85"
    ```
  ![cr5 robotiq85](../../.images/dobot_cr5_robotiq85.png)

* With ChangingTek AG2F90-C Gripper
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5 type:="AG2F90-C-Soft"
  ```
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5 type:="AG2F90-C"
  ```
  ![cr5 ag2f90-c](../../.images/dobot_cr5_ag2f90-c.png)

## 3. OCS2 Demo

### 3.1 Official OCS2 Mobile Manipulator Demo

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch manipulator_ocs2.launch.py robot_name:=cr5
```

![cr5 ocs2](../../.images/dobot_cr5_ocs2.gif)

### 3.2 OCS2 Arm Controller Demo

* Mock Components
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py  type:=AG2F90-C-Soft
  ```
* Gazebo
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=gz type:=AG2F90-C-Soft
  ```
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=gz type:=AG2F120S
  ```
* Isaac Sim
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=isaac type:=AG2F90-C-Soft
  ```

## 4. Real Dobot CR5 Deploy

* Compile Dobot ROS2 package
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to cr_robot_ros2 dobot_bridge --symlink-install
  ```
* Compile topic-based-ros2-control
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to topic_based_ros2_control --symlink-install
  ```
* Config Robot IP
  `192.168.5.38`
* Launch Dobot ROS2 Driver
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch cr5_description dobot_bringup_ros2.launch.py 
  ```
* Launch Dobot ROS2 Control
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=real type:=AG2F90-C-Soft
  ```
    ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=real type:=empty
  ```