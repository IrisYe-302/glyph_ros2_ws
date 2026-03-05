from setuptools import setup

package_name = 'go2_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
