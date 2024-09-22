from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robotics_trial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools','rclpy',
        'std_msgs',],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publisher=robotics_trial.map_publisher:main',
            'start_node_publisher=robotics_trial.start_node_publish:main',
            'goal_node_publisher=robotics_trial.goal_node_publish:main',
            'path_compute=robotics_trial.final_path:main'

        ],
    },
)
