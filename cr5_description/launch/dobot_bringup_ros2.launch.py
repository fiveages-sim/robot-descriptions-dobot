from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 暴露参数：与 quick_start / ocs2 demo 一致
    robot = LaunchConfiguration('robot')
    hardware = LaunchConfiguration('hardware')
    gripper_type = LaunchConfiguration('type')

    # 获取 vla_http_bridge 配置文件路径
    vla_http_config = os.path.join(
        get_package_share_directory('vla_http_bridge'),
        'config',
        'vla_http_bridge.yaml'
    )
    
    # OCS2 Arm Controller demo launch
    ocs2_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ocs2_arm_controller'), 'launch'),
            '/demo.launch.py'
        ]),
        launch_arguments=[
            ('robot', robot),
            ('type', gripper_type),
            ('hardware', hardware),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='cr5',
            description='Robot model (e.g. cr5 / cr5_dual)'
        ),
        DeclareLaunchArgument(
            'type',
            default_value='AG2F90-C-Soft',
            description='Gripper/type (e.g. AG2F90-C-Soft / vla39)'
        ),
        DeclareLaunchArgument(
            'hardware',
            default_value='mock_components',
            description='Hardware mode (mock_components / real)'
        ),
        # VLA HTTP Web API桥接节点
        launch_ros.actions.Node(
            package='vla_http_bridge',
            executable='vla_http_bridge',
            name='vla_http_bridge',
            output='screen',
            parameters=[vla_http_config],
            emulate_tty=True,
        ),
        
        # OCS2 Arm Controller (MPC控制器、RViz、状态发布器等)
        ocs2_demo_launch,
    ])
