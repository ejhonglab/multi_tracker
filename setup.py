from setuptools import setup

setup(
name='MultiTrackerAnalysis',
version='0.0.1',
install_requires=['hdf5', 'progressbar'],
author='Floris van Breugel',
author_email='floris@caltech.edu',
packages = ['multi_tracker_analysis'],
license='BSD',
description='Analysis scripts for data collected with multi tracker',
long_description=open('README.md').read(),
)
