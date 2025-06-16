from setuptools import setup

package_name = 'klipper_ros2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aswan',
    maintainer_email='aswanas393@gmail.com',
    description='A ROS 2 bridge for Klipper via Moonraker.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'klipper_bridge = klipper_ros2_bridge.klipper_bridge_node:main',
        ],
    },
)
