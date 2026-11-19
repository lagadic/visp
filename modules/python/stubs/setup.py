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

import sys
from pathlib import Path
from setuptools import setup

# The stubs generator is shared with the CMake install (INSTALL_PYTHON_BINDINGS=ON)
sys.path.insert(0, str(Path(__file__).parent.absolute()))
from generate_stubs import StubsGenerator

base_path = Path(__file__).parent.absolute()
stubs_generator = StubsGenerator(base_path / 'gen', base_path / 'visp')
stubs_generator.run()

setup(
  packages=['visp'],
  zip_safe=False,
  package_data = {
    'visp': stubs_generator.package_data
  },
)
