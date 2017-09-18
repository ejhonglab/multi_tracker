# not using this for now, because catkin not currently equipped to prevent egg folder
# from being created in this directory. TODO fix? necessary after "build"?
#from setuptools import setup
from distutils.core import setup

setup(
    name='MultiTrackerAnalysis',
    version='0.0.1',
    author='Floris van Breugel',
    author_email='floris@caltech.edu',
    scripts=['multi_tracker_analysis/trajectory_editor'],
    packages=['multi_tracker_analysis'],
    license='BSD',
    description='Analysis scripts for data collected with multi tracker',
    long_description=open('README.md').read(),
)

# only supported by setuptools
#install_requires=['numpy', 'h5py', 'progressbar', 'pandas', 'scipy', \
#    'sympy', 'pyqtgraph'],
