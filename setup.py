"""Setup script for the Spot Minimal Driver package."""

from glob import glob
from setuptools import find_packages, setup

package_name = 'spot_minimal_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools',
                      'bosdyn-client',
                      'bosdyn-api',
                      'bosdyn-core'],
    zip_safe=True,
    maintainer='Yixiang Gao',
    maintainer_email='ygao@missouri.edu',
    description='A minimal ROS 2 driver for Boston Dynamics Spot robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_driver_node = spot_minimal_driver.spot_minimal_driver:main',
        ],
    },
)
