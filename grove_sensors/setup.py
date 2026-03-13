from setuptools import setup
import os
from glob import glob

setup(
    name='grove_sensors',
    version='0.1.0',
    packages=['grove_sensors'],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/grove_sensors']),
        ('share/grove_sensors', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'imu_node = grove_sensors.imu_node:main',
            'ultrasonic_node = grove_sensors.ultrasonic_node:main',
        ],
    },
)
