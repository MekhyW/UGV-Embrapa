import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Path to the existing launch file
    rtabmap_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py']
    )

    realsense_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('realsense2_camera'), '/launch/rs_launch.py']
    )

    nav2_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py']
    )

    realsense_params_file = os.path.join(get_package_share_directory('osr_autonomous'), 'realsense_params', 'params.yaml')

    nav2_params_file = os.path.join(get_package_share_directory('osr_autonomous'), 'nav2_params', 'params.yaml')
    nav2_map_file = os.path.join(get_package_share_directory('osr_autonomous'), 'nav2_params', 'map.yaml')

    #Path to URDF
    urdf_file = os.path.join(get_package_share_directory('osr_gazebo'), 'urdf', 'osr.urdf.xacro')

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
            arguments=["0", "0", "0", "0", "0", "0", 'base_link', 'camera_link'],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{'publish_tf': False, 'use_mag': False}],
            remappings=[('/imu/data_raw', '/camera/camera/imu')],
        ),
        IncludeLaunchDescription(
            rtabmap_launch_file,
            launch_arguments={
                'rgb_topic': '/camera/camera/infra1/image_rect_raw',
                'depth_topic': '/camera/camera/depth/image_rect_raw',
                'camera_info_topic': '/camera/camera/infra1/camera_info',
                'qos': '1',
                'namespace': '',
                #'queue_size': '200',
                'rviz': 'false',
                'rtabmap_viz': 'false',
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
    ])
