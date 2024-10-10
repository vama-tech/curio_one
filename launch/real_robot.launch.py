import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart,OnExecutionComplete
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():



    package_name='curio_one'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )])
    )

    camera_detection = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','cameradetection.launch.py'
            )])
)
    

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[{'robot_description':robot_description},
                controller_params_file]
    )
    delayed_controller_manager = TimerAction(period=2.0, actions=[controller_manager])


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[diff_drive_spawner],
    )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[joint_broad_spawner],
    )
    )

    sensor_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster"],

    )

    delayed_sensor_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[sensor_broad_spawner],
        )
    )

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )])
    )
    range_file = Node(
        package =package_name,
        executable="range.py",
    )
    range1_file = Node(
        package =package_name,
        executable="1_range.py",
    )

    range2_file = Node(
        package =package_name,
        executable="2_range.py",
    )
    
    range4_file = Node(
        package =package_name,
        executable="4_range.py",
    )
    range5_file = Node(
        package =package_name,
        executable="5_range.py",
    )

    range6_file = Node(
        package =package_name,
        executable="6_range.py",
    )
    range7_file = Node(
        package =package_name,
        executable="7_range.py",
    )
    range8_file = Node(
        package =package_name,
        executable="8_range.py",
    )
    battery_file = Node(
        package =package_name,
        executable="battery_stat.py",
    )

    
    delayed_joystick = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[joystick],
    )
    )
    lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','sllidar_a1_launch.py'
            )]))
    delayed_lidar = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[lidar]))


    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    bno055 = Node(
                package='bno055',
                executable='bno055',
                parameters=[os.path.join(config_dir, 'bno055_params_i2c.yaml')]

    )
    # madgwick = Node(
    #             package='imu_filter_madgwick',
    #             executable='imu_filter_madgwick_node',
    #             name='imu_filter',
    #             output='screen',
    #             parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
    #         )
    imu_params_file = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')
    ekf_config = Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[imu_params_file],
                remappings=[("odom", LaunchConfiguration('odom_topic', default='/odom'))]

            )
    # imu = Node(
    #             package="ros2_icm20948",
    #             executable="icm20948_node",
    #             name="icm20948_node",
    #             parameters=[
    #                 {"i2c_address": 0x69},
    #                 {"frame_id": "base_link"},
    #                 {"pub_rate": 50},
    #             ],
    #         )

    return LaunchDescription([
        rsp,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad,
        delayed_sensor_broad_spawner,
        delayed_joystick,
        range_file,
        range1_file,
        range2_file,
        range4_file,
        range5_file,
        range6_file,
        range7_file,
        range8_file,
        battery_file,
        # delayed_lidar,
        bno055,
        # imu,
        # madgwick,
        ekf_config,
        # camera_detection

    ])