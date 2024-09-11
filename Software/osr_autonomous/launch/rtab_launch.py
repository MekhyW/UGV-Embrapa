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

    #Path to URDF
    urdf_file = os.path.join(get_package_share_directory('osr_gazebo'), 'urdf', 'osr.urdf.xacro')

    # Include the existing launch file and override some arguments
    return LaunchDescription([
        IncludeLaunchDescription(
            realsense_launch_file,
            launch_arguments={
                'enable_rgbd': 'true',
                'enable_sync': 'true',
                'enable_color': 'true',
                'enable_depth': 'true',
                'align_depth.enable': 'true',
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
        IncludeLaunchDescription(
            rtabmap_launch_file,
            launch_arguments={
                'rgb_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'qos': '1',
                'queue_size': '200',
                'rviz': 'false',
                'rtabmap_viz': 'false',
                'frame_id': 'camera_link',
                'Odom/Strategy': '1',
                'Vis/CorType': '1',
                'OdomF2M/MaxSize': '1000',
                'Vis/MaxFeatures': '600',
                'GFTT/MinDistance': '10',
                'rtabmap_args': '--delete_db_on_start',
                #'rgbd_topic': '/camera/camera/rgbd',
                #'rgbd_sync': 'true',
                #'imu_topic': '/imu/data',
                #'subscribe_scan': 'true',
                #'scan_topic': '/scan',
            }.items()
        ),
    ])
