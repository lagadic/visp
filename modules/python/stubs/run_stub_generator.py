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

from shutil import copy, copytree
from pathlib import Path
import subprocess
import sys
import argparse
import os

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--output-root', type=str, help='Path where to save the stubs')

  args = parser.parse_args()
  output_root = Path(args.output_root)
  output_root.mkdir(exist_ok=True)
  bin_folder = Path(sys.executable).parent
  subprocess.run(['stubgen',  '-o', str(output_root.absolute()), '--ignore-errors', '--include-docstrings', '-p', 'visp._visp'], check=True)
  subprocess.run(['stubgen',  '-o', str(output_root.absolute()), '--ignore-errors', '--include-docstrings', '--parse-only', '-p', 'visp.python'], check=True)


  all_submodule_names = ['python']

  # Generate stubs for the bindings (C++ side) and mock it so that they appear in the true 'visp' package
  cpp_stubs = Path(output_root / 'visp' / '_visp')
  target_path = Path('./visp')
  target_path.mkdir(exist_ok=True)
  for pyi_file in cpp_stubs.iterdir():
    if pyi_file.name.endswith('.pyi'):
      if pyi_file.stem != '__init__':

        all_submodule_names.append(pyi_file.stem)
        submodule_stub_folder = target_path / pyi_file.stem
        submodule_stub_folder.mkdir(exist_ok=True)
        copy(pyi_file, submodule_stub_folder / '__init__.pyi') # Copy replace old files

  # Add stubs for python part
  python_stubs_path = Path(output_root / 'visp' / 'python')
  copytree(str(python_stubs_path), str(target_path / 'python'), dirs_exist_ok=True)

  # Generate a complete list of modules for the __init__ stubs
  init_stubs_path = target_path / '__init__.pyi'
  with open(init_stubs_path, 'w') as init_stubs_file:
    # init_stubs_file.write(f'from ._visp import *{os.linesep}')

    for submodule in all_submodule_names:
      init_stubs_file.write(f'from ._visp import {submodule}{os.linesep}')
