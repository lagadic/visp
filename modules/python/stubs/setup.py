#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP Python bindings stubs
#
#############################################################################

from pathlib import Path
from setuptools import setup
from setuptools.command.install import install
import subprocess
import os
import sys
from shutil import copy, copytree

# Your custom post-install command
class StubsGenerator():
  def __init__(self):
    print('Running stubs generator')
    self.output_root = Path(__file__).parent.absolute() / 'gen'
    self.target_path = Path(Path(__file__).parent.absolute() / 'visp')
    self.output_root.mkdir(exist_ok=True)
    self.package_data = []


  def run(self):
    self.test_import_package()
    self.generate_stubs()
    self.rearrange_stubs()
    self.build_package_data()

  def test_import_package(self):
    print('Trying to import visp package...')
    res = subprocess.run([sys.executable, '-c', '\"import visp\"'], check=True)
    res.check_returncode()

  def generate_stubs(self):
    '''
    Call Mypy's tool to generate stubs in an temporary folder.
    CPP stubs are first generated, then the python specific ones
    '''
    options = ['--ignore-errors', '--include-docstrings', '--verbose']
    subprocess.run(['stubgen',  '-o', str(self.output_root.absolute()), *options, '-p', 'visp._visp'], check=True)
    subprocess.run(['stubgen',  '-o', str(self.output_root.absolute()), *options, '--parse-only', '-p', 'visp.python'], check=True)

  def rearrange_stubs(self):

    all_submodule_names = ['python']

    # Generate stubs for the bindings (C++ side) and mock it so that they appear in the true 'visp' package
    cpp_stubs = Path(self.output_root / 'visp' / '_visp')
    self.target_path.mkdir(exist_ok=True)
    for pyi_file in cpp_stubs.iterdir():
      if pyi_file.name.endswith('.pyi'):
        if pyi_file.stem != '__init__':

          all_submodule_names.append(pyi_file.stem)
          submodule_stub_folder = self.target_path / pyi_file.stem
          submodule_stub_folder.mkdir(exist_ok=True)
          submodules_stubs = submodule_stub_folder / '__init__.pyi'
          copy(pyi_file, submodules_stubs) # Copy replace old files

    # Add stubs for python part
    python_stubs_path = Path(self.output_root / 'visp' / 'python')
    copytree(str(python_stubs_path), str(self.target_path / 'python'), dirs_exist_ok=True)


    # Generate a complete list of modules for the __init__ stubs
    init_stubs_path = self.target_path / '__init__.pyi'
    with open(init_stubs_path, 'w') as init_stubs_file:
      # init_stubs_file.write(f'from ._visp import *{os.linesep}')

      for submodule in all_submodule_names:
        init_stubs_file.write(f'from . import {submodule}{os.linesep}')

  def build_package_data(self):
    def package_data_rec(module: Path, l):
      for submodule_path in module.iterdir():
        if submodule_path.name.endswith('.pyi'):
          l.append(str(submodule_path.absolute()))
        elif submodule_path.is_dir():
          package_data_rec(submodule_path, l)
        else:
          print(f'Found file {submodule_path} but it will not be added to package data!')
    package_data_rec(self.target_path, self.package_data)

stubs_generator = StubsGenerator()
stubs_generator.run()

setup(
  packages=['visp'],
  zip_safe=False,
  package_data = {
    'visp': stubs_generator.package_data
  },
)
