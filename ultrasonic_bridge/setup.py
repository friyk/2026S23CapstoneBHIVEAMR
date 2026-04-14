from setuptools import setup

package_name = 'ultrasonic_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/realsense_obstacle.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='zelin',
    maintainer_email='zelin@local',
    description='ESP32 HC-SR04 ultrasonic bridge + safety stop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ultrasonic_node = ultrasonic_bridge.ultrasonic_node:main',
            'safety_stop_node = ultrasonic_bridge.safety_stop_node:main',
        ],
    },
)
