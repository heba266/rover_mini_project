from setuptools import setup, find_packages

package_name = 'planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['planner', 'planner.*']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hager',
    maintainer_email='hager@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'planner_node = planner.planner:main',  # must point to main()
        ],
    },
)
