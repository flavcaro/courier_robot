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
    ],
    install_requires=[
        'setuptools',
        'py_trees>=2.2.0',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Courier navigation with mission behavior tree',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_controller = courier_nav.courier_controller:main',
            'spawner = courier_nav.world_spawner:main',
        ],
    },
)