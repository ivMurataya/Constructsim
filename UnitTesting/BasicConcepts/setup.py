# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robot_control'],
    package_dir={'': 'src'},
)

setup(**setup_args)
"""
We are going to generate the necessary package structure so that we can import 
this code later in our unit tests.
Well basically, the setup.py is a Python file that usually 
tells you that the module/package containing it has been packaged 
and distributed with Distutils, which is the standard tool for distributing Python Modules.

The third and final step will be to do a small modification in our CMakeLists.txt file. 
You will have to look for the following section at the top of the file:
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
...and uncomment the last line

And that's it! At the end, if you've followed all the steps properly, you should have your package structured in the following way
"""
