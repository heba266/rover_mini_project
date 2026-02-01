from setuptools import setup
import os
from glob import glob

package_name = 'wheel_odometry_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Wheel odometry node for robot localization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_converter = wheel_odometry_pkg.turtlebot4_converter:main',
            'wheel_odometry_node = wheel_odometry_pkg.wheel_odometry_node:main',
            'turtlebot3_converter = wheel_odometry_pkg.turtlebot3_converter:main',
            'localization_node = wheel_odometry_pkg.wheel_odometry_node_cooked:main',

        ],
    },
)