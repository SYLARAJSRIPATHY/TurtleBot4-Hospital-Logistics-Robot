from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable, TimerAction, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, shutil, math

def generate_launch_description():
    world  = LaunchConfiguration('world')
    model  = LaunchConfiguration('model')          
    gui    = LaunchConfiguration('gui')
    x      = LaunchConfiguration('x')
    y      = LaunchConfiguration('y')
    z      = LaunchConfiguration('z')
    yaw    = LaunchConfiguration('yaw')
    use_nav2   = LaunchConfiguration('use_nav2')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    seed_pose  = LaunchConfiguration('seed_initial_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_scan_filter = LaunchConfiguration('use_scan_filter')

    scan_topic_gz = LaunchConfiguration('scan_topic_gz')

    pkg_share  = get_package_share_directory('autonomous_bot')
    tb4_share  = get_package_share_directory('turtlebot4_ignition_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    worlds_dir   = os.path.join(pkg_share, 'worlds')
    rviz_cfg     = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')
    nav2_params  = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    laser_filters_yaml = os.path.join(pkg_share, 'config', 'laser_filters.yaml')

    set_res_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{worlds_dir}:{os.environ.get('GZ_SIM_RESOURCE_PATH','')}"
    )

    tb4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_share, 'launch', 'turtlebot4_ignition.launch.py')
        ),
        launch_arguments={
            'world': world, 'model': model, 'gui': gui,
            'x': x, 'y': y, 'z': z, 'yaw': yaw,
            'use_sim_time': use_sim_time
        }.items()
    )

    bridges = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            [scan_topic_gz, TextSubstitution(text='@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')],
            '/model/turtlebot4/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/turtlebot4_standard/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            (scan_topic_gz, '/scan_raw'),
            ('/model/turtlebot4/odometry', '/odom'),
            ('/model/turtlebot4_standard/odometry', '/odom'),
        ],
        output='screen'
    )

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_node',
        parameters=[laser_filters_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen'
    )

    static_laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_laser_frame_fix',
        arguments=['0','0','0','0','0','1.5708','base_link','turtlebot4/rplidar_link/rplidar'],
        output='screen'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
            'log_level': 'INFO',
        }.items()
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    def pick_terminal_prefix():
        for term in ('gnome-terminal','konsole','xterm'):
            import shutil
            if shutil.which(term):
                return f'{term} -e'
        return ''
    teleop = Node(
        package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
        name='teleop_keyboard', prefix=pick_terminal_prefix(),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    def assemble(context, *args, **kwargs):
        actions = [set_res_path, tb4, bridges, static_laser_tf]

        relay_cmd_vel_tb4 = Node(
            package="topic_tools", executable="relay", name="relay_cmd_vel_to_tb4",
            arguments=["/cmd_vel", "/model/turtlebot4/cmd_vel"],
            parameters=[{'use_sim_time': use_sim_time}], output="screen",
        )
        relay_cmd_vel_tb4_std = Node(
            package="topic_tools", executable="relay", name="relay_cmd_vel_to_tb4_standard",
            arguments=["/cmd_vel", "/model/turtlebot4_standard/cmd_vel"],
            parameters=[{'use_sim_time': use_sim_time}], output="screen",
        )
        actions += [relay_cmd_vel_tb4, relay_cmd_vel_tb4_std]

        if use_scan_filter.perform(context).lower() in ('1','true','yes'):
            actions.append(scan_filter)
        if use_nav2.perform(context).lower() in ('1','true','yes'):
            actions.append(TimerAction(period=5.0, actions=[nav2]))
        if use_rviz.perform(context).lower() in ('1','true','yes'):
            actions.append(TimerAction(period=7.0, actions=[rviz]))
        if use_teleop.perform(context).lower() in ('1','true','yes'):
            actions.append(TimerAction(period=8.0, actions=[teleop]))

        if seed_pose.perform(context).lower() in ('1','true','yes'):
            qz = 0.0  
            qw = 1.0
            msg = "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation:{z: 0.0, w: 1.0}}}}"
            for delay in (6.0, 7.0, 8.0):
                actions.append(TimerAction(
                    period=delay,
                    actions=[ExecuteProcess(
                        cmd=['ros2','topic','pub','--once','/initialpose',
                             'geometry_msgs/PoseWithCovarianceStamped', msg],
                        output='screen')]))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='hospital'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('gui',   default_value='true'),
        DeclareLaunchArgument('x',   default_value='4.6'),
        DeclareLaunchArgument('y',   default_value='-9.6'),
        DeclareLaunchArgument('z',   default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='-1.5708'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'hospital.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed_initial_pose', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='false'),
        DeclareLaunchArgument('scan_topic_gz',
            default_value='/world/hospital/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan'),
        DeclareLaunchArgument('use_scan_filter', default_value='false'), 
        OpaqueFunction(function=assemble),
    ])