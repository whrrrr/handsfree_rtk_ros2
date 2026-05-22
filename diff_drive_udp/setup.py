from setuptools import find_packages, setup
from glob import glob


package_name = 'diff_drive_udp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whr',
    maintainer_email='2603612944@qq.com',
    description='UDP bridge from ROS 2 cmd_vel to a simple differential drive controller.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'diff_drive_udp = diff_drive_udp.diff_drive_udp:main',
        ],
    },
)
