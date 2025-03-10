from setuptools import setup
import os
from glob import glob


package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/my_custom_crazy_costmap'), glob('config/my_custom_crazy_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/straightline_planner'), glob('config/straightline_planner/*.yaml')),
        (os.path.join('share', package_name, 'config/wavyline_planner'), glob('config/wavyline_planner/*.yaml')),
        (os.path.join('share', package_name, 'config/pure_pursuit_controller'), glob('config/pure_pursuit_controller/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tgrip',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
