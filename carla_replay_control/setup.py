"""
Setup for carla_replay_control
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(packages=['carla_replay_control'], package_dir={'': 'src'})

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'carla_replay_control'
    setup(
        name=package_name,
        version='0.0.1',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Valentin Rusche',
        maintainer_email='valentin.rusche@udo.edu',
        description='CARLA replay control for ROS2 bridge',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': ['carla_replay_control = carla_replay_control.carla_replay_control:main'],
        },
        package_dir={'': 'src'},
    )
