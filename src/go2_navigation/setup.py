from glob import glob
import os

package_name = 'go2_navigation'

from setuptools import setup, find_packages

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Navigation utilities for Unitree Go2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'location_subscriber = go2_navigation.location_subscriber:main',
            'initial_pose_publisher = go2_navigation.initial_pose_publisher:main',
            'goal_tolerance_marker = go2_navigation.goal_tolerance_marker:main',
            'sim_fall_recovery = go2_navigation.sim_fall_recovery:main',
        ],
    },
)
