import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package paths
    pkg_share_dir = get_package_share_directory('simulation_pkg')          # string path
    pkg_share_path = get_package_share_path('simulation_pkg')             # pathlib path

    # URDF/Xacro + mesh directory
    path_to_urdf = pkg_share_path / 'description' / 'robot_urdf.xacro'
    models_dir = os.path.join(pkg_share_dir, 'models')

    # robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', str(path_to_urdf),
                    ' ', 'mesh_dir:=', models_dir
                ]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    # World file
    world = os.path.join(pkg_share_dir, 'worlds', 'industrial-warehouse.sdf')

    # Ensure Gazebo can resolve model:// URIs relative to package share (optional but fine to keep)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = pkg_share_dir if existing == '' else (pkg_share_dir + os.pathsep + existing)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world]}.items(),
    )

    # Spawn robot in Gazebo (from /robot_description)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robot1",
            "-topic", "/robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "1.4",
        ],
        output="screen",
    )

    # Bridge Gazebo <-> ROS
    bridge_params = os.path.join(pkg_share_dir, 'params', 'ros_gz_bridge.yaml')
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # -------------------------
    # Scan fixer (ROS-side)
    # /scan -> /scan_fixed (+ fills scan_time/time_increment, optionally overrides stamp)
    # -------------------------
    scan_fixer = Node(
        package='simulation_pkg',
        executable='scan_fixer_node',
        name='scan_fixer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'in_topic': '/scan',
            'out_topic': '/scan_fixed',
            'override_stamp': True,
        }]
    )

    odom_fixer = Node(
        package='simulation_pkg',
        executable='odom_fixer_node',
        name='odom_fixer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'in_topic': '/diff_controller/odom',
            'out_topic': '/odom_fixed',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
        }]
    )

    # Start scan_fixer after the bridge starts (so it has an input topic)
    #start_scan_fixer_after_bridge = RegisterEventHandler(
    #    OnProcessExit(
    #        target_action=start_gazebo_ros_bridge_cmd,
    #        on_exit=[scan_fixer],
    #    )
    #)

    # Controllers: spawn AFTER the robot is created to avoid timing issues.
    # Start joint_state_broadcaster first, then diff_controller.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '20',
        ],
        output='screen'
    )

    diff_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '20',
        ],
        output='screen'
    )

    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_diff_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_controller_spawner],
        )
    )

    # twist_mux -> diff_controller cmd_vel
    twist_mux_params = os.path.join(pkg_share_dir, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', 'diff_controller/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),

        set_gz_resource_path,
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        start_gazebo_ros_bridge_cmd,

        # Start scan fixer (after bridge)
        #start_scan_fixer_after_bridge,
        # controller startup (gated)
        scan_fixer,
        odom_fixer,
        start_jsb_after_spawn,
        start_diff_after_jsb,

        twist_mux,
    ])
