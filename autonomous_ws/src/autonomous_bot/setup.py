from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ROS 2 package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch, config, maps, rviz, worlds
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*pgm')),   # map images
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sylaraj',
    maintainer_email='sylaraj@todo.todo',
    description='Navigation bringup package for autonomous_bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'room_navigator = autonomous_bot.room_navigator:main',
            'room_navigatorup = autonomous_bot.room_navigatorup:main',
        ],
    },
)
