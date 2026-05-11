from glob import glob
from setuptools import find_packages, setup

package_name = 'handsfree_rtk'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/usb_rules', glob('usb_rules/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='handsfree',
    maintainer_email='handsfree@todo.todo',
    description='ROS 2 driver for HandsFree RTK UM98x receivers.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'ros_driver_um98x = handsfree_rtk.ros_driver_um98x:main',
            'ros_driver_um98x_ntrip = handsfree_rtk.ros_driver_um98x_ntrip:main',
        ],
    },
)
