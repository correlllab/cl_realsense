from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'cl_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/Rviz', glob('Rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='correlllabhumanoid@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc_acc = cl_realsense.scripts.point_cloud_accumulator:main',
            'another_node = cl_realsense.scripts.another_node:main',
        ],
    },
)
