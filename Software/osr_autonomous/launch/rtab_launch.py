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
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '1',
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
                #'rgbd_topic': '/camera/camera/rgbd',
                #'rgbd_sync': 'true',
                'qos': '1',
                'rviz': 'true',
                'frame_id': 'camera_link',
                'imu_topic': '/camera/camera/imu',
                #'subscribe_scan': 'true',
                #'scan_topic': '/scan',
            }.items()
        ),
    ])
