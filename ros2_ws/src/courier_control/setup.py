from setuptools import find_packages, setup

package_name = 'courier_control'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Control package for courier robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pid_controller = courier_control.pid_controller:main',
        ],
    },
)