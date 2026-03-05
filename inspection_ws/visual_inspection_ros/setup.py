from setuptools import setup
import os
from glob import glob

package_name = 'visual_inspection_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.bt_nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/action', glob('action/*.action')),
        (f'share/{package_name}/msg', glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dinethra',
    description='Visual inspection IBVS pipeline as ROS2 nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Run individual nodes:
            'camera_node         = visual_inspection_ros.camera_node:main',
            'servo_node          = visual_inspection_ros.servo_node:main',
            'ibvs_action_server  = visual_inspection_ros.ibvs_action_server:main',
        ],
    },
)
