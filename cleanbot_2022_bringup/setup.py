from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages

d = generate_distutils_setup(
    packages=['coral_usb'],
    package_dir={'': 'python'},
)

setup(**d)
