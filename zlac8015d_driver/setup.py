from glob import glob
from setuptools import find_packages, setup


package_name = 'zlac8015d_driver'

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
    description='ROS 2 RS485 Modbus driver for two ZLAC8015D dual wheel drivers.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'zlac8015d_cmd_vel = zlac8015d_driver.zlac8015d_cmd_vel:main',
            'zlac8015d_position_jog = zlac8015d_driver.zlac8015d_position_jog:main',
            'zlac8015d_scan_ids = zlac8015d_driver.zlac8015d_scan_ids:main',
        ],
    },
)
