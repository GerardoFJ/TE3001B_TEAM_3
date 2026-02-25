from setuptools import find_packages, setup

package_name = 'xarm_perturbations'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nezih-niegu',
    maintainer_email='nezih-niegu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perturbation_injector = xarm_perturbations.perturbation_injector:main',
            'circle_maker = xarm_perturbations.circle_maker:main',
            'trajectory_generator = xarm_perturbations.trajectory_generator:main',
            'position_controller = xarm_perturbations.position_controller:main',
            'evaluator = xarm_perturbations.evaluator:main',
        ],
    },

)
