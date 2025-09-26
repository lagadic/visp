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
# ViSP Python bindings module
#
#############################################################################

import sys
import os
# import os
# sys.path.append(os.path.dirname(__file__))
# print(sys.path)


try:
  from ._visp import *
  from ._visp import __dict__ as cpp_extension_dict
except ImportError:
  import platform
  if platform.system() == "Windows": # On windows import can fail because DLLs are not found in the default search paths
    from .windows_dll_manager import get_dll_paths, build_directory_manager
    # Use the context to clean up the PATH/dll directories after the import (no namespace pollution)
    with build_directory_manager() as dll_dir_manager:
      for p in get_dll_paths():
        dll_dir_manager.add_dll_directory(p)
      from ._visp import *
      from ._visp import __dict__ as cpp_extension_dict
  else:
    raise ImportError('Could not import ViSP python bindings')


# Fake module names
for k in cpp_extension_dict:
  from types import ModuleType
  if isinstance(_visp.__dict__[k], ModuleType):
    sys.modules[f'{__name__}.{k}'] = _visp.__dict__[k]


import visp.python
