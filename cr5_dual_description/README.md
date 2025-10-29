# CR5 Dual Arm Description

This package contains the description files for a dual-arm workstation equipped with two Dobot CR5 collaborative robot arms. The workstation features left and right CR5 arms mounted on a custom body platform.

## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to cr5_dual_description --symlink-install
```

## 2. Visualize the Robot

* Without gripper
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual
    ```
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual collider:=simple
    ```
* With Robotiq 85 Gripper
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual type:="robotiq85"
    ```

* With ChangingTek AG2F90-C Gripper
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual type:="AG2F90-C-Soft"
  ```

## 3. OCS2 Demo
### 3.1 Official OCS2 Mobile Manipulator Demo

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch manipulator_ocs2.launch.py robot_name:=cr5_dual
```

### 3.2 OCS2 Arm Controller Demo
* Mock Component
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual type:="AG2F90-C-Soft"
  ```
* Gazebo
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=gz robot:=cr5_dual world:=warehouse type:="AG2F90-C-Soft"
  ```
* Isaac Sim
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=isaac robot:=cr5_dual type:="AG2F90-C-Soft"
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
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
  ```
    ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real
  ```