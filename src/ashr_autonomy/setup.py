import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ashr_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install the config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='youssef.nagy073@gmail.com',
    description='Autonomy FSM for ASHR Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_server = ashr_autonomy.target_server:main',
            'autonomy_node = ashr_autonomy.autonomus_node.autonomy_node:main',
        ],
    },
)