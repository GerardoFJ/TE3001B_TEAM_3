import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'xarm_bilateral_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'isaac_sim'),
         glob(os.path.join('isaac_sim', '*.py'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'force_sensor     = xarm_bilateral_teleop.force_sensor_node:main',
            'master_admittance = xarm_bilateral_teleop.master_node:main',
            'slave_impedance   = xarm_bilateral_teleop.slave_node:main',
            'keyboard_teleop   = xarm_bilateral_teleop.keyboard_teleop:main',
            'traj_to_js        = xarm_bilateral_teleop.trajectory_to_joint_state:main',
        ],
    },
)
