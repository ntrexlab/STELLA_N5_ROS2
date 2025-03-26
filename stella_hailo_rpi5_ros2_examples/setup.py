from setuptools import setup
from glob import glob

package_name = 'stella_hailo_rpi5_ros2_examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml'] + glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NTREX LAB',
    maintainer_email='lab@ntrex.co.kr',
    description='ROS2 package bassed on hailo-rpi5-example for STELLA.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hailo_ros2_detection_node = stella_hailo_rpi5_ros2_examples.hailo_ros2_detection_node:main'
        ],
    },
)