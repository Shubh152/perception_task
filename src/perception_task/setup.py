from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'perception_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name,'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name,'perception_task'), glob('perception_task/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubh',
    maintainer_email='shubh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_node = perception_task.depth_node:main',
        ],
    },
)