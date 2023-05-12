#!/usr/bin/env python

from distutils.core import setup
setup(
    name='megapose_server',
    version='0.1',
    description='Megapose TCP server',
    package_dir={'megapose_server': '.'},
    author='Samuel Felton',
    author_email='samuel.felton@irisa.fr',
)

# setup()