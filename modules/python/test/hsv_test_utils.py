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
# Helpers shared by the HSV image filter tests.
# Python port of modules/core/test/image/hsvUtils.h
#
#############################################################################

import numpy as np

from visp.core import ImageGray, ImageBool, ImageHSVucFS, ImageHSVucLS, ImageHSVd

SIZE = 11

# Name of the HSV image type -> (ViSP image class, numpy dtype)
HSV_TYPES = {
  'HSVucFS': (ImageHSVucFS, np.uint8),
  'HSVucLS': (ImageHSVucLS, np.uint8),
  'HSVd': (ImageHSVd, np.float64),
}

PATTERNS = ['square', 'inversed', 'horizontal', 'vertical', 'pyramid']


def _make_patterns():
  '''
  Build the synthetic 11x11 patterns of vpHSVTests::vpInputDataset, both as double values (used for vpHSV<double>)
  and unsigned char values (used for vpHSV<unsigned char> and gray images), and the associated boolean mask.
  '''
  patterns_d = {name: np.zeros((SIZE, SIZE), dtype=np.float64) for name in PATTERNS}
  patterns_uc = {name: np.zeros((SIZE, SIZE), dtype=np.uint8) for name in PATTERNS}
  mask = np.zeros((SIZE, SIZE), dtype=bool)

  for r in range(2, 9):
    for c in range(2, 9):
      if r in (2, 3, 7, 8) or c in (2, 3, 7, 8):
        mask[r, c] = True
      if c <= 5:
        val, val_inversed = 0.1 * (c - 1), 0.1 * (6 - c)
        val_c, val_c_inversed = c - 1, 6 - c
      else:
        val, val_inversed = 0.1 * (9 - c), 0.1 * (c - 4)
        val_c, val_c_inversed = 9 - c, c - 4
      if r <= 5:
        val_pyramid, val_c_pyramid = val + 0.1 * (r - 2), val_c + r - 2
      else:
        val_pyramid, val_c_pyramid = val + 0.1 * (8 - r), val_c + 8 - r

      for patterns, v, v_inv, v_pyr in [(patterns_d, val, val_inversed, val_pyramid),
                                        (patterns_uc, val_c, val_c_inversed, val_c_pyramid)]:
        if r in (2, 8):
          patterns['square'][r, c] = v
          patterns['inversed'][r, c] = v_inv
          patterns['square'][c, r] = v
          patterns['inversed'][c, r] = v_inv
        patterns['horizontal'][r, c] = v
        patterns['vertical'][c, r] = v
        patterns['pyramid'][r, c] = v_pyr

  return patterns_d, patterns_uc, mask


_PATTERNS_D, _PATTERNS_UC, _MASK = _make_patterns()


def make_mask():
  '''
  Boolean mask of the dataset, as a visp.core.ImageBool.
  '''
  return ImageBool(_MASK.copy())


def make_gray_image(pattern):
  '''
  Gray-level version of the pattern, as a visp.core.ImageGray.
  '''
  return ImageGray(_PATTERNS_UC[pattern].copy())


def make_hsv_image(hsv_type, pattern):
  '''
  HSV version of the pattern, all the channels (H, S, V) having the same value.
  '''
  ImageType, dtype = HSV_TYPES[hsv_type]
  values = _PATTERNS_D[pattern] if dtype == np.float64 else _PATTERNS_UC[pattern]
  return ImageType(np.ascontiguousarray(np.repeat(values[:, :, None], 3, axis=2).astype(dtype)))
