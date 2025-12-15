# CR5 Cartesian Controllers 

## 4. Cartesian Controllers
### 4.1 Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to cartesian_compliance_controller cartesian_controller_handles --symlink-install
```

The `cartesian_controller.launch.py` from `robot_common_launch` supports three controller types: `motion`, `force`, and `compliance`. Use the `controller_type` parameter to select the desired controller.

* Install Force-Torque Sensor Broadcaster
```bash
sudo apt-get install ros-jazzy-force-torque-sensor-broadcaster
```

### 4.2 Cartesian Motion Controller
* Mock Component
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5
  ```
* Real Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=real type:=ft-90c
  ```

### 4.3 Cartesian Force Controller
* Real Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=real type:=ft-90c controller_type:=force
  ```

### 4.4 Cartesian Compliance Controller
* Real Robot
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=real type:=ft-90c controller_type:=compliance
  ```