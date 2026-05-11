from glob import glob
from setuptools import find_packages, setup

package_name = 'gps_waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whr',
    maintainer_email='2603612944@qq.com',
    description='Simple GPS waypoint follower for outdoor ground robots.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'clear_waypoints = gps_waypoint_nav.clear_waypoints:main',
            'capture_waypoint = gps_waypoint_nav.capture_waypoint:main',
            'waypoint_follower = gps_waypoint_nav.waypoint_follower:main',
        ],
    },
)
