from setuptools import setup
#Import dependencies
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
'''
The objective of this code is to install the launch files. For example, with the package named my_package, this will install all the launch files from the launch/ folder, into ~/ros2_ws/install/my_package/share/my_package/.
'''
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          # You are adding an entry point to the script you wrote earlier, simple.py. Enter the executable name from the launch file
            'simple_node = my_package.simple:main'
        ],
    },
)
