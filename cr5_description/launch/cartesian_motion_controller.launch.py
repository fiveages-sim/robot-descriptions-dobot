"""
Cartesian Motion Controller Launch File for CR5 Robot

This launch file starts the cartesian_motion_controller for CR5 robot.
It uses the common controller_manager launch file for hardware configuration,
similar to ocs2_arm_controller demo launch.

Usage:
    # With real hardware
    ros2 launch cr5_description cartesian_motion_controller.launch.py hardware:=real type:=AG2F90-C-Soft

    # With Gazebo simulation
    ros2 launch cr5_description cartesian_motion_controller.launch.py hardware:=gz type:=AG2F90-C-Soft world:=dart

    # With mock components (for testing)
    ros2 launch cr5_description cartesian_motion_controller.launch.py hardware:=mock_components

    # With interactive marker handle (RViz visualization)
    ros2 launch cr5_description cartesian_motion_controller.launch.py use_rviz:=true use_handle:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations.get('robot', 'cr5')
    robot_type = context.launch_configurations.get('type', '')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'dart')
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    use_handle = context.launch_configurations.get('use_handle', 'true').lower() == 'true'

    # 根据 hardware 参数自动判断是否使用仿真时间
    use_sim_time = hardware in ['gz', 'isaac']

    # 构建 remappings 字符串（格式: "from1:to1;from2:to2"）
    # 将控制器话题映射到统一的 target_frame（不带前导斜杠，与 cartesian_controller_simulation 一致）
    remappings_list = []
    if use_handle:
        remappings_list.append('motion_control_handle/target_frame:target_frame')
    remappings_list.append('cartesian_motion_controller/target_frame:target_frame')
    remappings_str = ';'.join(remappings_list)

    # 使用通用的 controller manager launch 文件
    # 它处理了机器人描述生成、robot_state_publisher、Gazebo 启动等
    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_common_launch'), 'launch'),
            '/controller_manager.launch.py',
        ]),
        launch_arguments=[
            ('robot', robot_name),
            ('type', robot_type),
            ('use_sim_time', str(use_sim_time)),
            ('world', world),
            ('hardware', hardware),  # 传递硬件类型，controller_manager 会根据此参数自动判断是否使用 Gazebo
            ('remappings', remappings_str),  # 传递 remappings 参数
        ],
    )

    # Cartesian motion controller spawner
    # Remap target_frame to unified /target_frame topic
    cartesian_motion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_motion_controller'],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/cartesian_motion_controller/target_frame', '/target_frame')
        ]
    )

    # Motion control handle spawner
    # Remap target_frame to unified /target_frame topic
    motion_control_handle_spawner = None
    if use_handle:
        motion_control_handle_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['motion_control_handle'],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/motion_control_handle/target_frame', '/target_frame')
            ]
        )

    # RViz node
    rviz_node = None
    if use_rviz:
        cr5_description_pkg = get_package_share_directory('cr5_description')
        rviz_config = os.path.join(
            cr5_description_pkg,
            'config',
            'rviz',
            'cr5.rviz'
        )
        # Fallback to default RViz if config doesn't exist
        if not os.path.exists(rviz_config):
            rviz_config = ""
        
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config] if rviz_config else [],
            parameters=[{"use_sim_time": use_sim_time}],
        )

    # Collect all nodes
    # controller_manager_launch 已经包含了：
    # - robot_state_publisher
    # - Gazebo 相关节点（如果 hardware=gz）
    # - ros2_control_node（如果 hardware!=gz）
    # - joint_state_broadcaster spawner
    nodes = [
        controller_manager_launch,
        cartesian_motion_controller_spawner,
    ]
    
    # 添加 motion_control_handle spawner（如果启用）
    if motion_control_handle_spawner:
        nodes.append(motion_control_handle_spawner)
    
    if rviz_node:
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    # Declare launch arguments
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='cr5',
        description='Robot name (default: cr5)'
    )

    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='mock_components',
        description='Hardware type: real, gz (Gazebo), mock_components, or isaac'
    )

    type_arg = DeclareLaunchArgument(
        'type',
        default_value='',
        description='Robot type/variant (e.g., AG2F90-C-Soft)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='dart',
        description='Gazebo world file name (only used when hardware=gz)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz visualization'
    )

    use_handle_arg = DeclareLaunchArgument(
        'use_handle',
        default_value='true',
        description='Whether to launch motion control handle (interactive marker)'
    )

    return LaunchDescription([
        robot_arg,
        hardware_arg,
        type_arg,
        world_arg,
        use_rviz_arg,
        use_handle_arg,
        OpaqueFunction(function=launch_setup),
    ])

