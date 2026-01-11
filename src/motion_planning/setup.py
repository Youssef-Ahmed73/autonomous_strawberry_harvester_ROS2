from setuptools import setup

package_name = 'motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='youssef.nagy073@gmail.com',
    description='Motion planning package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'motion_planner = motion_planning.motion_planner:main',
        ],
    },
)
