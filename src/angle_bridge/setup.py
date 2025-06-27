from setuptools import setup

package_name = 'angle_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=['angle_bridge'],
    install_requires=['setuptools==59.6.0', 'rclpy==1.12.0'],
    zip_safe=True,
    maintainer='Hinata Koizumi',
    maintainer_email='example@example.com',
    description='PX4 fan PWM → Gazebo joint コマンド橋渡しノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'angle_bridge_node = angle_bridge.angle_bridge:main',
        ],
    },
    data_files=[],
)
