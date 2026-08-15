from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_chain'

setup(
    name=package_name,
    version='3.0.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'),
         glob('configs/*.json')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anonymous Authors',
    maintainer_email='anonymous@example.com',
    description='PX4 ROS 2 swarm chain controllers and logging tools.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_controller = swarm_chain.mission_controller:main',
            'chain_controller = swarm_chain.chain_controller:main',
            'state_broadcaster = swarm_chain.state_broadcaster:main',
            'state_logger = swarm_chain.state_logger:main',
            'metrics_chain_entropy = swarm_chain.metrics_chain_entropy:main',
            'attack_manager = swarm_chain.attack_manager:main',
        ],
    },
)
