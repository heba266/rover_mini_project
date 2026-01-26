from setuptools import find_packages, setup

package_name = 'costmap_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud-fathy',
    maintainer_email='mahmoudfathi443@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "local_costmap = costmap_pkg.local_costmap:main",
            "local_costmap_odom = costmap_pkg.local_costmap_odom:main",

        ],
    },
)
