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
# ViSP Python bindings test for the HSV gradient filters of visp.core.ImageFilter.
# Python port of modules/core/test/image/testHSVGradient.cpp
#
#############################################################################

import numpy as np
import pytest

from visp.core import ImageDouble, ImageFloat
from visp.core import ImageFilter

from hsv_test_utils import HSV_TYPES, PATTERNS, make_hsv_image, make_mask

FILTER_TYPES = [ImageFilter.CANNY_GBLUR_SOBEL_FILTERING, ImageFilter.CANNY_GBLUR_SCHARR_FILTERING]


def smoothing_kernel(filter_type):
  if filter_type == ImageFilter.CANNY_GBLUR_SOBEL_FILTERING:
    return np.array([1., 2., 1.]) / 8.
  if filter_type == ImageFilter.CANNY_GBLUR_SCHARR_FILTERING:
    return np.array([3., 10., 3.]) / 32.
  raise ValueError('Wrong type of filtering')


def reference_gradient(I, mask, filter_type):
  '''
  Reference implementation of the HSV gradient: the differences of the V channel along one axis are smoothed
  along the other axis. Borders are set to 0, as well as the pixels that are not in the mask.
  '''
  V = I.numpy()[:, :, 2].astype(np.float64)
  h, w = V.shape
  f = smoothing_kernel(filter_type)

  diff_x = np.zeros((h, w))
  diff_x[:, :-1] = V[:, 1:] - V[:, :-1]
  diff_y = np.zeros((h, w))
  diff_y[:-1, :] = V[1:, :] - V[:-1, :]

  GIx = np.zeros((h, w))
  GIy = np.zeros((h, w))
  for d in (-1, 0, 1):
    GIx[1:-1, 1:-1] += f[d + 1] * (diff_x[1 + d:h - 1 + d, 0:w - 2] + diff_x[1 + d:h - 1 + d, 1:w - 1])
    GIy[1:-1, 1:-1] += f[d + 1] * (diff_y[0:h - 2, 1 + d:w - 1 + d] + diff_y[1:h - 1, 1 + d:w - 1 + d])

  if mask is not None:
    keep = mask.numpy()
    GIx[~keep] = 0.
    GIy[~keep] = 0.
  return GIx, GIy


@pytest.mark.parametrize('use_mask', [False, True], ids=['no_mask', 'mask'])
@pytest.mark.parametrize('filter_type', FILTER_TYPES, ids=['sobel', 'scharr'])
@pytest.mark.parametrize('nb_thread', [1, 2])
@pytest.mark.parametrize('pattern', PATTERNS)
@pytest.mark.parametrize('hsv_type', list(HSV_TYPES.keys()))
@pytest.mark.parametrize('OutType', [ImageDouble, ImageFloat])
def test_hsv_gradient_filter(OutType, hsv_type, pattern, nb_thread, filter_type, use_mask):
  I = make_hsv_image(hsv_type, pattern)
  mask = make_mask() if use_mask else None
  GIx_ref, GIy_ref = reference_gradient(I, mask, filter_type)
  atol = 1e-3

  GIx, GIy = OutType(), OutType()
  ImageFilter.gradientFilter(I, GIx, GIy, nb_thread, mask, filter_type)
  np.testing.assert_allclose(GIx.numpy(), GIx_ref, atol=atol)
  np.testing.assert_allclose(GIy.numpy(), GIy_ref, atol=atol)

  # Same results when computing each direction separately
  GIx, GIy = OutType(), OutType()
  ImageFilter.gradientFilterX(I, GIx, nb_thread, mask, filter_type)
  ImageFilter.gradientFilterY(I, GIy, nb_thread, mask, filter_type)
  np.testing.assert_allclose(GIx.numpy(), GIx_ref, atol=atol)
  np.testing.assert_allclose(GIy.numpy(), GIy_ref, atol=atol)


@pytest.mark.parametrize('hsv_type', list(HSV_TYPES.keys()))
def test_hsv_gradient_filter_default_args(hsv_type):
  I = make_hsv_image(hsv_type, 'pyramid')
  GIx, GIy = ImageDouble(), ImageDouble()
  # Default: all threads, no mask, Sobel filtering
  ImageFilter.gradientFilter(I, GIx, GIy)
  GIx_ref, GIy_ref = reference_gradient(I, None, ImageFilter.CANNY_GBLUR_SOBEL_FILTERING)
  np.testing.assert_allclose(GIx.numpy(), GIx_ref, atol=1e-3)
  np.testing.assert_allclose(GIy.numpy(), GIy_ref, atol=1e-3)


@pytest.mark.parametrize('pattern, expect_x, expect_y', [
  ('horizontal', True, False),
  ('vertical', False, True),
  ('pyramid', True, True),
])
def test_hsv_gradient_direction(pattern, expect_x, expect_y):
  '''
  On the horizontal (resp. vertical) pattern, the gradient along X (resp. Y) is expected on the whole square
  surface, while the gradient along the other axis is only expected on the borders.
  '''
  I = make_hsv_image('HSVd', pattern)
  GIx, GIy = ImageDouble(), ImageDouble()
  ImageFilter.gradientFilter(I, GIx, GIy, 1)
  inner = (slice(4, 7), slice(4, 7))
  assert np.abs(GIx.numpy()[inner]).max() > 1e-6 if expect_x else np.allclose(GIx.numpy()[inner], 0.)
  assert np.abs(GIy.numpy()[inner]).max() > 1e-6 if expect_y else np.allclose(GIy.numpy()[inner], 0.)
