from setuptools import setup

package_name = 'manus_to_inspire'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'manus_to_inspire_node = manus_to_inspire.manus_refiner:main',
        ],
    },
)
