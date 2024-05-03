from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Corrected the usage of glob() here

        # Add your other data file configurations here...
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karankapoor',
    maintainer_email='karan.kapoor@uwaterloo.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_rl = drone_rl.drone_rl:main'
        ],
    },
)
