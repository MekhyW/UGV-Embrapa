import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the existing launch file
    rtabmap_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py']
    )

    realsense_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('realsense2_camera'), '/launch/rs_launch.py']
    )

    nav2_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py']
    )

    nmea_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('nmea_navsat_driver'), '/launch/nmea_serial_driver.launch.py']
    )

    realsense_params_file = os.path.join(get_package_share_directory('osr_autonomous'), 'realsense_params', 'params.yaml')

    nav2_params_file = os.path.join(get_package_share_directory('osr_autonomous'), 'nav2_params', 'params.yaml')
    nav2_map_file = os.path.join(get_package_share_directory('osr_autonomous'), 'nav2_params', 'map.yaml')

    #Path to URDF
    urdf_file = os.path.join(get_package_share_directory('osr_gazebo'), 'urdf', 'osr.urdf.xacro')


    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value='rosbag',
        description='Directory to save bag files'
    )

    bag_dir = LaunchConfiguration('bag_dir')

    # Include the existing launch file and override some arguments
    return LaunchDescription([
        IncludeLaunchDescription(
            realsense_launch_file,
            launch_arguments={
                'config_file': realsense_params_file,
                'enable_sync': 'true',
                'enable_color': 'false',
                'enable_depth': 'true',
                'enable_infra1': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '1',
                #'enable_infra2': 'true',
            }.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[urdf_file],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_camera_link',
            arguments=["0.13", "0", "0.175", "0", "-0.52", "0", 'base_link', 'camera_link'],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{'publish_tf': False, 'use_mag': False}],
            remappings=[('/imu/data_raw', '/camera/camera/imu')],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_tf',
            output='screen',
            parameters=[{
                        "broadcast_cartesian_transform": True,
                        #"frequency": 30.0, 
                        #"delay": 3.0,
                        #"magnetic_declination_radians": 0.0,
                        #"yaw_offset": 0.0,
                        #"broadcast_utm_transform": "true",
                        #"publish_filtered_gps": "true",
                        #"use_odometry_yaw": "true",
                        "wait_for_datum": False,
                        "zero_altitude": True,
                        #"datum": [38.161491, -122.4546443, 0.0] # pre-set datum if needed, [lat, lon, yaw]
                    }],
            remappings=[('odometry/filtered', 'odom'), ('gps/fix', 'fix'), ('/imu', '/imu/data')],
        ),
        Node(
            package='osr_autonomous',
            executable='gps_follower',
            name="gps_follower",
            output = "screen",
        ),
        IncludeLaunchDescription(
            nmea_launch_file,
        ),
        IncludeLaunchDescription(
            rtabmap_launch_file,
            launch_arguments={
                'rgb_topic': '/camera/camera/infra1/image_rect_raw',
                'depth_topic': '/camera/camera/depth/image_rect_raw',
                'camera_info_topic': '/camera/camera/infra1/camera_info',
                'qos': '1',
                'wait_for_transform': '0.3',
                'output': 'none',
                'namespace': '',
                'queue_size': '20',
                'rviz': 'false',
                'rtabmap_viz': 'false',
                'approx_sync': 'true',
                'frame_id': 'base_footprint',
                #'publish_tf_odom': 'true',
                #'rtabmap_args': '--delete_db_on_start --Vis/MaxFeatures 1000 --Grid/3D false --Rtabmap/DetectionRate 10 --Reg/Force3DoF true --Kp/DetectorStrategy 2 --Vis/FeatureType 2',
                'rtabmap_args': '--delete_db_on_start --Kp/DetectorStrategy 2 --Vis/FeatureType 2 --Vis/MaxFeatures 1000 --Rtabmap/DetectionRate 1',
                #'rtabmap_args': '--delete_db_on_start --Vis/CorType 1 --Reg/Force3DoF true',
                #'odom_args': '--Odom/Strategy 1  --Reg/Force3DoF',
                #'odom_args': '--Reg/Force3DoF',
                #'rgbd_topic': '/camera/camera/rgbd',
                #'rgbd_sync': 'true',
                'imu_topic': '/imu/data',
                #'subscribe_scan': 'true',
                #'scan_topic': '/scan',
            }.items()
        ),
        IncludeLaunchDescription(
            nav2_launch_file,
            launch_arguments={
                'params_file': nav2_params_file,
                'map': nav2_map_file,
            }.items()
        ),
        bag_dir_arg,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_dir],
            output='screen'
        ),
    ])
