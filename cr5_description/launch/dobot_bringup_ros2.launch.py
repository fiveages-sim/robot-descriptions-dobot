from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


dobot_ros2_params = [
    {"robot_ip_address": "192.168.5.38"},
    {"robot_type": "cr5"},
    {"trajectory_duration": 0.3},
    {"robot_node_name": "dobot_bringup_ros2"},
    {"robot_number": 1},
]

def generate_launch_description():
    # 获取dobot_command_bridge配置文件路径
    dobot_command_config = os.path.join(
        get_package_share_directory('dobot_bridge'),
        'config',
        'dobot_command_bridge.yaml'
    )
    
    # 获取dobot_flask_bridge配置文件路径
    dobot_flask_config = os.path.join(
        get_package_share_directory('dobot_bridge'),
        'config',
        'dobot_flask_bridge.yaml'
    )
    
    # OCS2 Arm Controller demo launch
    ocs2_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ocs2_arm_controller'), 'launch'),
            '/demo.launch.py'
        ]),
        launch_arguments=[
            ('robot', 'cr5'),
            ('type', 'AG2F90-C-Soft'),
            ('hardware', 'real'),
        ],
    )

    return LaunchDescription([
        # Dobot底层驱动节点
        launch_ros.actions.Node(
            package='cr_robot_ros2',
            executable='cr_robot_ros2_node',
            name="dobot_bringup_ros2",
            output='screen',
            parameters=dobot_ros2_params
        ),
        
        # Dobot关节指令桥接节点
        launch_ros.actions.Node(
            package='dobot_bridge',
            executable='dobot_command_bridge',
            name='dobot_command_bridge',
            output='screen',
            parameters=[dobot_command_config],
            emulate_tty=True,
        ),
        
        # Dobot Flask Web API桥接节点
        launch_ros.actions.Node(
            package='dobot_bridge',
            executable='dobot_flask_bridge',
            name='dobot_flask_bridge',
            output='screen',
            parameters=[dobot_flask_config],
            emulate_tty=True,
        ),
        
        # OCS2 Arm Controller (MPC控制器、RViz、状态发布器等)
        ocs2_demo_launch,
    ])
