#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
# ViSP Python bindings stubs generator.
#
# Used by setup.py when the stubs are installed with pip, and called directly
# by CMake when the bindings are installed with the CMake install target
# (INSTALL_PYTHON_BINDINGS=ON):
#
#   python generate_stubs.py --output-root <tmp dir> --target <stubs dir> [--bindings-path <dir>]
#
#############################################################################

import argparse
import os
import subprocess
import sys
from pathlib import Path
from shutil import copy, copytree
from typing import List, Optional


class StubsGenerator():
  def __init__(self, output_root: Path, target_path: Path, bindings_path: Optional[Path] = None):
    '''
    :param output_root: Folder where mypy's stubgen writes the raw stubs
    :param target_path: Folder where the rearranged stubs ('visp' package) are written
    :param bindings_path: Optional folder containing the 'visp' package to generate the stubs for.
      It is prepended to PYTHONPATH, so that the stubs can be generated from the build tree.
    '''
    print('Running stubs generator')
    self.output_root = Path(output_root).absolute()
    self.target_path = Path(target_path).absolute()
    self.bindings_path = Path(bindings_path).absolute() if bindings_path is not None else None
    self.output_root.mkdir(parents=True, exist_ok=True)
    self.package_data: List[str] = []

  def run(self):
    self.generate_stubs()
    self.rearrange_stubs()
    self.build_package_data()

  def generate_stubs(self):
    '''
    Call Mypy's tool to generate stubs in an temporary folder.
    CPP stubs are first generated, then the python specific ones
    '''
    env = os.environ.copy()
    if self.bindings_path is not None:
      env['PYTHONPATH'] = os.pathsep.join(p for p in [str(self.bindings_path), env.get('PYTHONPATH', '')] if p)
    # Equivalent to the 'stubgen' console script, but bound to the current interpreter.
    # 'python -m mypy.stubgen' cannot be used: mypy wheels are compiled with mypyc ("No code object available")
    stubgen = [sys.executable, '-c', 'import sys; from mypy.stubgen import main; sys.exit(main())', '-o', str(self.output_root)]
    subprocess.run(stubgen + ['--ignore-errors', '--include-docstrings', '-p', 'visp._visp'], check=True, env=env)
    subprocess.run(stubgen + ['--ignore-errors', '--include-docstrings', '--parse-only', '-p', 'visp.python'], check=True, env=env)

  def rearrange_stubs(self):

    all_submodule_names = ['python']

    # Generate stubs for the bindings (C++ side) and mock it so that they appear in the true 'visp' package
    cpp_stubs = Path(self.output_root / 'visp' / '_visp')
    if not cpp_stubs.is_dir():
      raise RuntimeError(f'No stubs were generated for visp._visp in {cpp_stubs}: '
                         'the module could not be imported (see the stubgen output above). '
                         'Check that "import visp" works with the same interpreter and PYTHONPATH.')
    self.target_path.mkdir(parents=True, exist_ok=True)
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


def main():
  parser = argparse.ArgumentParser(description='Generate the stubs of the ViSP Python bindings')
  parser.add_argument('--output-root', type=Path, required=True, help='Folder for the raw stubs generated by stubgen')
  parser.add_argument('--target', type=Path, required=True, help='Folder where the stubs of the visp package are written')
  parser.add_argument('--bindings-path', type=Path, default=None, help='Folder containing the visp package, prepended to PYTHONPATH')
  args = parser.parse_args()
  StubsGenerator(args.output_root, args.target, args.bindings_path).run()


if __name__ == '__main__':
  main()
