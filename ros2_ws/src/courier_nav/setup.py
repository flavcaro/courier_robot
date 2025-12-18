from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'courier_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm') + glob('maps/*.png')),
    ],
    install_requires=[
        'setuptools',
        'py_trees>=2.2.0',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Courier navigation with Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav2_mission_controller = courier_nav.nav2_mission_controller:main',
            'spawner = courier_nav.world_spawner:main',
            'apriltag_localizer = courier_nav.apriltag_localizer:main',
        ],
    },
)