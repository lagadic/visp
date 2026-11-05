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
# ViSP Python bindings test for imgproc functions taking an optional mask
#
#############################################################################

import numpy as np

from visp.core import ImageGray, ImageRGBa, ImageBool, RGBa
from visp.imgproc import equalizeHistogram, gammaCorrection, GAMMA_MANUAL, GAMMA_RGB, GAMMA_HSV


def make_gray_image(h=32, w=48):
  data = np.tile(np.linspace(50, 150, w, dtype=np.uint8), (h, 1))
  return ImageGray(data)

def make_rgba_image(h=32, w=48):
  I = ImageRGBa(h, w, RGBa(0, 0, 0, 255))
  rgba = I.numpy()
  ramp = np.linspace(50, 150, w, dtype=np.uint8)
  for c in range(3):
    rgba[:, :, c] = ramp
  return I

def make_mask(h=32, w=48):
  M = ImageBool(h, w, False)
  m = M.numpy()
  m[:, : w // 2] = True
  return M

def test_equalize_histogram_gray():
  I = make_gray_image()
  ref = I.numpy().copy()

  # Out-of-place, no mask
  I2 = ImageGray()
  equalizeHistogram(I, I2)
  assert I2.getRows() == I.getRows() and I2.getCols() == I.getCols()
  np.testing.assert_array_equal(I.numpy(), ref)  # input untouched
  assert I2.numpy().min() < ref.min() and I2.numpy().max() > ref.max()

  # In-place with explicit None mask gives the same result
  I3 = ImageGray(ref.copy())
  equalizeHistogram(I3, p_mask=None)
  np.testing.assert_array_equal(I3.numpy(), I2.numpy())

  # With a mask
  I4 = ImageGray()
  equalizeHistogram(I, I4, make_mask())
  assert not np.array_equal(I4.numpy(), I2.numpy())

def test_gamma_correction_gray():
  I = make_gray_image()
  ref = I.numpy().copy()

  I2 = ImageGray()
  gammaCorrection(I, I2, 2.0)
  np.testing.assert_array_equal(I.numpy(), ref)
  assert not np.array_equal(I2.numpy(), ref)

  I3 = ImageGray(ref.copy())
  gammaCorrection(I3, 2.0, GAMMA_MANUAL, None)
  np.testing.assert_array_equal(I3.numpy(), I2.numpy())

  I4 = ImageGray()
  gammaCorrection(I, I4, 2.0, GAMMA_MANUAL, make_mask())
  assert I4.getRows() == I.getRows() and I4.getCols() == I.getCols()

def test_gamma_correction_rgba():
  I = make_rgba_image()
  ref = I.numpy().copy()

  I2 = ImageRGBa()
  gammaCorrection(I, I2, 2.0)
  np.testing.assert_array_equal(I.numpy(), ref)
  assert not np.array_equal(I2.numpy(), ref)

  I3 = ImageRGBa(ref.copy())
  gammaCorrection(I3, 2.0, GAMMA_RGB, GAMMA_MANUAL, None)
  np.testing.assert_array_equal(I3.numpy(), I2.numpy())

  for color_handling in [GAMMA_RGB, GAMMA_HSV]:
    I4 = ImageRGBa()
    gammaCorrection(I, I4, 2.0, color_handling, GAMMA_MANUAL, make_mask())
    assert I4.getRows() == I.getRows() and I4.getCols() == I.getCols()
