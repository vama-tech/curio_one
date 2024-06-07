import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='curio_one',
    version='0.0.0',
    packages=find_packages(
        include=('curio_one', 'curio_one.*')),
)
