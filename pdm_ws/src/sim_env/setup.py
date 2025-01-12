from setuptools import find_packages, setup
from setuptools.command.install import install
import subprocess
import glob
import os

package_name = 'sim_env'

class InstallGymEnvsUrdf(install):
    def run(self):
        # TODO: Check if this is the best way to do this. 
        subprocess.check_call(['git', 'submodule', 'update', '--init', '--recursive'])
        subprocess.check_call(['pip3', 'install', 'urdfenvs'])
        subprocess.check_call(['pip3', 'install', 'urdfpy'])
        subprocess.check_call(['pip3', 'install', '-e', './sim_env/include/gym_envs_urdf/'])
        subprocess.check_call(['pip3', 'install', 'keyboard'])
        install.run(self)

gym_envs_urdf_files = [
    file for file in glob.glob('sim_env/include/gym_envs_urdf/**/*', recursive=True)
    if os.path.isfile(file)  # Ensure only files are included
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['sim_env', 'sim_env.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/gym_envs_urdf', gym_envs_urdf_files),
        ('share/' + package_name + '/launch', [
            'launch/pdm_project.launch.xml'
        ]),
        ('share/' + package_name + '/resource', [
            'resource/map1.json','resource/map2.json','resource/map3.json','resource/map4.json',
        ]),
    ],
    install_requires=['setuptools', 'urdfenvs', 'keyboard', 'numpy'],
    zip_safe=True,
    maintainer='juaber',
    maintainer_email='jbernalmedina@tudelft.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panda_env = sim_env.src.env_node:main',
        ],
    },
    cmdclass={'install': InstallGymEnvsUrdf},
)
