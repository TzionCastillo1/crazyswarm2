import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    #load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    gz_server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'gz_server.yaml'
    )

    with open(gz_server_yaml, 'r') as ymlfile:
        gz_server_yaml_contents = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [gz_server_yaml_contents["/crazyflie_server"]["ros__parameters"]] + [{"use_sim_time": True}]

     # teleop params
    teleop_params = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_crazyflie_description = get_package_share_directory('crazyflie_description')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={ 'gz_args': os.path.join(pkg_crazyflie_description, 'worlds', 'crazyflie_world.sdf')}.items(),
        )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU', 
                   '/crazyflie/gazebo/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/crazyflie/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        remappings=[
            ('clock', 'clock'),
            ('model/crazyflie/odometry', 'cf/model/crazyflie/odometry'),
            ('imu', '/cf/imu'),
            ('/crazyflie/gazebo/command/motor_speed', 'cf/crazyflie/gazebo/command/motor_speed'),
        ],
        output='screen'
    )

    # RQT
    rqt = Node(
            package='rqt_topic',
            executable='rqt_topic',
            arguments=['-t']
            #condition=IfCondition(LaunchConfiguration('rqt'))
        )
    teleop =  Node(
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('takeoff', 'cf/takeoff'),
                ('land', 'cf/land'),
                ('cmd_full_state', 'cf/cmd_full_state'),
                ('cmd_vel_legacy', 'cf/cmd_vel_legacy'),
                ('notify_setpoints_stop', 'cf/notify_setpoints_stop'),
            ],
            parameters=[teleop_params]
        )
    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('crazyflie'), 'config', 'config.rviz')],
            parameters=[{
                "use_sim_time": True,
            }]
        )

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='sim'),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','sim'),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters=server_params,
        ),
        bridge,
        gz_sim,
        rqt,
        #teleop,
        rviz
       
    ])