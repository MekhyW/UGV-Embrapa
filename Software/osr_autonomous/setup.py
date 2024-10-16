from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'osr_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'nav2_params'), glob(os.path.join('nav2_params', '*.yaml'))),
        (os.path.join('share', package_name, 'realsense_params'), glob(os.path.join('realsense_params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='katri',
    maintainer_email='rkatri20033@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_follower = osr_autonomous.gps_follower:main',
        ],
    },
)
