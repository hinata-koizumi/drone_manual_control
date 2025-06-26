from setuptools import setup
import os
from glob import glob

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    python_requires='>=3.8',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join(os.path.dirname(__file__), '../../config/*.yaml'))),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py') if os.path.exists('launch') else []),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
    ],
    zip_safe=True,
    maintainer='drone_developer',
    maintainer_email='developer@example.com',
    description='Manual control system for predefined drone actions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_executor = manual_control.action_executor:main',
            'simple_simulator = manual_control.simple_simulator:main',
            'state_monitor = manual_control.state_monitor:main',
        ],
    },
) 