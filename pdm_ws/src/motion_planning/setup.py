from setuptools import find_packages, setup

package_name = 'motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['motion_planning', 'motion_planning.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/second.jpg'
        ]),
    ],
    install_requires=['setuptools', 'rclpy', 'sim_env'],
    zip_safe=True,
    maintainer='juaber',
    maintainer_email='jbernalmedina@tudelft.nl',
    description='Package for the Motion Planing computations',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts':[
            'motion_planner = motion_planning.src.motion_planner_node:main',
        ],
    },
)
