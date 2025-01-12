from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'],include=['control', 'control.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/joints_axis.pickle',
            'resource/joints_origin.pickle'
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='juaber',
    maintainer_email='jbernalmedina@tudelft.nl',
    description='Package used for the Control computations',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = control.src.control_node:main'
        ],
    },
    
    include_package_data=True,
)
