from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the existing launch file
    rtabmap_launch_file = PythonLaunchDescriptionSource(
        [FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py']
    )

    # Include the existing launch file and override some arguments
    return LaunchDescription([
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
