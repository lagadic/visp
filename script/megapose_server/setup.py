#!/usr/bin/env python

from distutils.core import setup
setup(
    name='megapose_server',
    version='0.1',
    description='Megapose TCP server',
    package_dir={'megapose_server': '.'},
    include_package_data=True,
    package_data={'': ['megapose_variables.json']},
    author='Samuel Felton',
    author_email='samuel.felton@irisa.fr',
)

# setup()