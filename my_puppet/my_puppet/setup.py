from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_puppet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),  # URDF files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),  # Launch files
        (os.path.join('share', package_name, 'puppetSounds'), glob('puppetSounds/*')),
    ],
    install_requires=[
    'setuptools',
    'rclpy',
    'pygame',
    'librosa',
    ],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Puppet Show Simulation in ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puppet_controller = my_puppet.puppet_controller:main',  # Corrected path
        ],
    },
)

