 
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['interactive_script'],
    package_dir={'': 'src'},
    scripts=['scripts/interactive_script']
)

setup(**d)
