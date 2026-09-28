from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pure_pursuit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'data'),
            glob('data/*')
        ),
        (
            os.path.join('share', package_name),
            ['README.md']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luishratacuco',
    maintainer_email='luishector.m.delrazo25@gmail.com',
    description=(
        'ROS 2 Pure Pursuit controller and trajectory analysis '
        'for a simulated ground vehicle.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_recorder = '
            'pure_pursuit_controller.path_recorder:main',
            'pure_pursuit_node = '
            'pure_pursuit_controller.pure_pursuit_node:main',
            'trajectory_recorder = '
            'pure_pursuit_controller.trajectory_recorder:main',
        ],
    },
)
