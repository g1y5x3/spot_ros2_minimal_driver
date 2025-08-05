from setuptools import find_packages, setup

package_name = 'nav_goal_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'tf2_ros', 'tf2_geometry_msgs'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='mspalding815@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'nav_goal_listener = nav_goal_listener.nav_goal_listener:main',
        ],
    },
)
