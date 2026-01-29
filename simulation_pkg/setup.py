from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulation_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament package index marker + package.xml (THIS MUST BE EXACTLY LIKE THIS)
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),

        # install runtime resources into install/.../share/simulation_pkg/...
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apostolos Apostolou',
    maintainer_email='aapost04@ucy.ac.cy',
    description='Package to simulate my final year thesis robot using Gazebo and ROS2 Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
