from setuptools import find_packages, setup

package_name = 'pedal_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'evdev'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'pedal_node = pedal_ros2.pedal_node:main',
        ],
    },
)
