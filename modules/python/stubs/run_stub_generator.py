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

from shutil import copy
from pathlib import Path
import subprocess
import sys
import argparse

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--output-root', type=str, help='Path where to save the stubs')

  args = parser.parse_args()
  output_root = Path(args.output_root)
  assert output_root.exists()
  bin_folder = Path(sys.executable).parent
  stubgen_entry_point = bin_folder / 'pybind11-stubgen'
  assert stubgen_entry_point.exists()

  subprocess.run([str(stubgen_entry_point), '-o', str(output_root.absolute()), '--ignore-all-errors', '_visp'], check=True)

  # Generate stubs for the bindings (C++ side) and mock it so that they appear in the true 'visp' package
  p = Path('./_visp')
  target_path = Path('./visp-stubs')
  target_path.mkdir(exist_ok=True)
  for pyi_file in p.iterdir():
    if pyi_file.name.endswith('.pyi'):
      copy(pyi_file, target_path / pyi_file.name) # Copy replace old files
