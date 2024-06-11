#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
# Display helpers for ViSP
#
#############################################################################

from typing import List, Optional

from visp.core import Display

VISP_DISPLAY_CLS_MAP = {
  'x': None,
  'opencv': None,
  'gtk': None,
  'win32': None,
  'gdi': None,
}

try:
  from visp.gui import DisplayX
  VISP_DISPLAY_CLS_MAP['x'] = DisplayX
except ImportError:
  pass

try:
  from visp.gui import DisplayOpenCV
  VISP_DISPLAY_CLS_MAP['opencv'] = DisplayOpenCV
except ImportError:
  pass

try:
  from visp.gui import DisplayGTK
  VISP_DISPLAY_CLS_MAP['gtk'] = DisplayGTK
except ImportError:
  pass

try:
  from visp.gui import DisplayWin32
  VISP_DISPLAY_CLS_MAP['win32'] = DisplayWin32
except ImportError:
  pass

try:
  from visp.gui import DisplayGDI
  VISP_DISPLAY_CLS_MAP['gdi'] = DisplayGDI
except ImportError:
  pass

VISP_DEFAULT_DISPLAY_PREFERENCE = ['x', 'opencv', 'gtk', 'win32', 'gdi']


def get_display(preferences: Optional[List[str]] = None) -> Optional[Display]:
  '''
  Get a new ViSP display instance, dependending on what display driver is available.

  :param preference: An optional list of preferred backends to use.
  The backends are tested in the order they are specified, and the first match is instanciated.
  If preference is None, a default list of display is used. This list contains all basic displays.
  The specified values are case insensitive and may include. See VISP_DEFAULT_DISPLAY_PREFERENCE for the available options

  :return: a new instance of a ViSP display if one of the requested backend has been found, None otherwise
  '''

  final_prefs: List[str] = preferences if preferences is not None else VISP_DEFAULT_DISPLAY_PREFERENCE
  for preference in final_prefs:
    pref_key = preference.lower()
    display_opt = VISP_DISPLAY_CLS_MAP.get(pref_key)
    if display_opt is not None:
      return display_opt()
  return None
