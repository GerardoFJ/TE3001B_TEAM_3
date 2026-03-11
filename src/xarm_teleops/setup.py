from setuptools import setup, find_packages

package_name = 'xarm_teleops'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'master = master_control:main',
            'slave  = slave_control:main',
        ],
    },
)
