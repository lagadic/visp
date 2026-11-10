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
# ViSP Python bindings test for the vpImageFilter namespace, exposed as the
# visp.core.ImageFilter submodule
#
#############################################################################

import types

import numpy as np
import pytest

from visp.core import ImageGray, ImageFloat, ImageDouble, ImageBool, ArrayDouble2D, ArrayFloat2D
from visp.core import ImageFilter


def make_gray_image(h=40, w=50):
  I = ImageGray(h, w, 0)
  I.numpy()[10:30, 15:35] = 200
  return I

def make_mask(h=40, w=50):
  M = ImageBool(h, w, False)
  M.numpy()[:, : w // 2] = True
  return M

def test_image_filter_is_submodule():
  assert isinstance(ImageFilter, types.ModuleType)
  import visp.core.ImageFilter as image_filter_module
  assert image_filter_module is ImageFilter

def test_enums_in_submodule():
  assert int(ImageFilter.CANNY_GBLUR_SOBEL_FILTERING) == 0
  assert int(ImageFilter.CANNY_GBLUR_SCHARR_FILTERING) == 1
  assert int(ImageFilter.CANNY_OPENCV_BACKEND) == 0
  assert int(ImageFilter.CANNY_VISP_BACKEND) == 1
  assert ImageFilter.CannyBackendType.CANNY_VISP_BACKEND == ImageFilter.CANNY_VISP_BACKEND
  name = ImageFilter.vpCannyBackendTypeToString(ImageFilter.CANNY_VISP_BACKEND)
  assert ImageFilter.vpCannyBackendTypeFromString(name) == ImageFilter.CANNY_VISP_BACKEND
  name = ImageFilter.vpCannyFiltAndGradTypeToStr(ImageFilter.CANNY_GBLUR_SCHARR_FILTERING)
  assert ImageFilter.vpCannyFiltAndGradTypeFromStr(name) == ImageFilter.CANNY_GBLUR_SCHARR_FILTERING

@pytest.mark.parametrize('ArrayType', [ArrayDouble2D, ArrayFloat2D])
def test_gaussian_kernels(ArrayType):
  size = 7
  for fn in [ImageFilter.getGaussianKernel, ImageFilter.getGaussianDerivativeKernel]:
    kernel = ArrayType()
    fn(kernel, size)
    assert kernel.getRows() == 1 and kernel.getCols() == (size + 1) // 2
    assert kernel.numpy().any()

  kernel = ArrayType()
  ImageFilter.getGaussianKernel(kernel, size, normalize=True)
  k = kernel.numpy()[0]
  # Central coefficient + symmetric coefficients
  assert k[0] + 2 * k[1:].sum() == pytest.approx(1.0, abs=1e-5)

def test_sobel_scharr_kernels():
  for fn, size in [(ImageFilter.getSobelKernelX, 5), (ImageFilter.getSobelKernelY, 5),
                   (ImageFilter.getScharrKernelX, 3), (ImageFilter.getScharrKernelY, 3)]:
    kernel = ArrayDouble2D()
    scale = fn(kernel, size)
    assert kernel.getRows() == size and kernel.getCols() == size
    assert scale > 0

def test_gaussian_blur():
  I = make_gray_image()
  for OutType in [ImageFloat, ImageDouble]:
    Iblur = OutType()
    ImageFilter.gaussianBlur(I, Iblur)
    assert Iblur.getRows() == I.getRows() and Iblur.getCols() == I.getCols()
    assert Iblur.numpy().any()

    Iblur_mask = OutType()
    ImageFilter.gaussianBlur(I, Iblur_mask, size=7, sigma=0., normalize=True, mask=make_mask())
    assert not np.array_equal(Iblur.numpy(), Iblur_mask.numpy())

def test_separable_filter_matches_gaussian_blur():
  I = make_gray_image()
  size = 7
  kernel = ArrayDouble2D()
  ImageFilter.getGaussianKernel(kernel, size)
  Isep = ImageDouble()
  ImageFilter.separableFilter(I, Isep, kernel)
  Iblur = ImageDouble()
  ImageFilter.gaussianBlur(I, Iblur, size)
  np.testing.assert_allclose(Isep.numpy(), Iblur.numpy(), atol=1e-6)

def test_filter_x_y():
  I = make_gray_image()
  kernel = ArrayDouble2D()
  ImageFilter.getGaussianKernel(kernel, 5)
  Ix = ImageDouble()
  Iy = ImageDouble()
  ImageFilter.filterX(I, Ix, kernel)
  ImageFilter.filterY(Ix, Iy, kernel)
  Isep = ImageDouble()
  ImageFilter.separableFilter(I, Isep, kernel)
  np.testing.assert_allclose(Iy.numpy(), Isep.numpy(), atol=1e-6)

def test_2d_filter():
  I = make_gray_image()
  kernel = ArrayDouble2D()
  ImageFilter.getSobelKernelX(kernel, 3)
  If = ImageDouble()
  ImageFilter.filter(I, If, kernel)
  assert If.getRows() == I.getRows() and If.numpy().any()
  Iu = ImageDouble()
  Iv = ImageDouble()
  ImageFilter.filter(I, Iu, Iv, kernel, convolve=True)
  assert Iu.numpy().any() and Iv.numpy().any()

def test_gradients():
  I = make_gray_image()
  dIx = ImageDouble()
  ImageFilter.getGradX(I, dIx)
  assert dIx.numpy().any()

  size = 7
  gaussian = ArrayDouble2D()
  derivative = ArrayDouble2D()
  ImageFilter.getGaussianKernel(gaussian, size)
  ImageFilter.getGaussianDerivativeKernel(derivative, size)

  dIy = ImageDouble()
  ImageFilter.getGradY(I, dIy, derivative)
  assert dIy.numpy().any()

  dIx_gauss = ImageDouble()
  dIy_gauss = ImageDouble()
  ImageFilter.getGradXGauss2D(I, dIx_gauss, gaussian, derivative)
  ImageFilter.getGradYGauss2D(I, dIy_gauss, gaussian, derivative, make_mask())
  assert dIx_gauss.numpy().any() and dIy_gauss.numpy().any()

def test_partial_derivatives_and_canny_threshold():
  I = make_gray_image()
  dIx = ImageDouble()
  dIy = ImageDouble()
  ImageFilter.computePartialDerivatives(I, dIx, dIy, filteringType=ImageFilter.CANNY_GBLUR_SCHARR_FILTERING,
                                        backend=ImageFilter.CANNY_VISP_BACKEND)
  assert dIx.numpy().any() and dIy.numpy().any()

  lower, upper = ImageFilter.computeCannyThreshold(I)
  assert 0. < lower < upper

def test_canny():
  I = make_gray_image()
  Ic = ImageFilter.canny(I, 5, 15., 3)
  assert Ic.getRows() == I.getRows() and Ic.getCols() == I.getCols()
  assert set(np.unique(Ic.numpy())).issubset({0, 255})
  assert Ic.numpy().any()

  Ic2 = ImageGray()
  ImageFilter.canny(I, Ic2, 5, -1., -1., 3, 1., 0.6, 0.8, True,
                    ImageFilter.CANNY_VISP_BACKEND, ImageFilter.CANNY_GBLUR_SOBEL_FILTERING, None)
  assert Ic2.numpy().any()
