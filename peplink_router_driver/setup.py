from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'peplink_router_driver'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(
            os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Iverach-Brereton',
    maintainer_email='civerachb@clearpathrobotics.com',
    description='ROS 2 driver for Peplink mobile routers',
    license='BSD',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'peplink_router_node = peplink_router_driver.peplink_router_node:main'
        ],
    },
)
