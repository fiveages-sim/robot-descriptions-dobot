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
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch manipulator.launch.py robot:=cr5_dual type:="AG2F90-C"
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
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual type:="AG2F90-C"
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

* Dual Arm
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C
  ```
* Only Left Arm
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=left_arm
  ```

## 5. Cartesian Controllers
### 5.1 Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to cartesian_compliance_controller cartesian_controller_handles --symlink-install
```

### 5.2 Cartesian Motion Controller
* Mock Component
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch cr5_description cartesian_motion_controller.launch.py
  ```
* Real Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch cr5_description cartesian_motion_controller.launch.py hardware:=real robot:=cr5_dual type:=left_arm
  ```

### 4.2 Cartesian Force Controller
* Real Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch cr5_description cartesian_force_controller.launch.py hardware:=real robot:=cr5_dual type:=left_arm
  ```