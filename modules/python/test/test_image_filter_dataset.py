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
# ViSP Python bindings test for visp.core.ImageFilter: 2D filtering, Sobel, separable filtering,
# median and Gaussian blur. The results are compared to numpy reference implementations that follow
# the OpenCV conventions used in the C++ test.
# Python port of modules/core/test/image-with-dataset/testImageFilter.cpp
#
#############################################################################

import os

import numpy as np
import pytest

from visp.core import ArrayDouble2D, ColVector, ImageDouble, ImageGray, ImageRGBa, IoTools
from visp.core import ImageFilter

try:
  from visp.io import ImageIo
except ImportError:
  ImageIo = None

DATASET_PATH = IoTools.getViSPImagesDataPath()

requires_dataset = pytest.mark.skipif(
  ImageIo is None or not DATASET_PATH or not os.path.exists(os.path.join(DATASET_PATH, 'Klimt', 'Klimt.pgm')),
  reason='ViSP dataset is not available, set VISP_INPUT_IMAGE_PATH environment variable')


def make_kernel(rows, cols):
  ''' Kernel filled with 1, 2, ..., rows * cols. '''
  return np.arange(1, rows * cols + 1, dtype=np.float64).reshape(rows, cols)


KERNELS = [make_kernel(2, 2), make_kernel(3, 3), make_kernel(2, 3)]
KERNEL_IDS = ['2x2', '3x3', '2x3']


def correlate(I, kernel, convolve=False):
  '''
  2D correlation (or convolution) of I with kernel, with the anchor at the kernel center as cv::filter2D() does.
  Only the pixels for which the kernel is fully inside the image are computed, others are set to NaN.
  '''
  I = np.asarray(I, dtype=np.float64)
  h, w = I.shape
  kr, kc = kernel.shape
  hr, hc = kr // 2, kc // 2
  if convolve:
    # Convolution: out[i, j] = sum K[a, b] I[i + hr - a, j + hc - b]
    kernel = kernel[::-1, ::-1]
    off_r, off_c = hr - (kr - 1), hc - (kc - 1)
  else:
    # Correlation: out[i, j] = sum K[a, b] I[i + a - hr, j + b - hc]
    off_r, off_c = -hr, -hc
  out = np.full((h, w), np.nan)
  i0, i1 = max(0, -off_r), min(h, h - (kr - 1 + off_r))
  j0, j1 = max(0, -off_c), min(w, w - (kc - 1 + off_c))
  acc = np.zeros((i1 - i0, j1 - j0))
  for a in range(kr):
    for b in range(kc):
      acc += kernel[a, b] * I[i0 + a + off_r:i1 + a + off_r, j0 + b + off_c:j1 + b + off_c]
  out[i0:i1, j0:j1] = acc
  return out


def check_results(ref, I, margin_rows, margin_cols, atol):
  ''' Compare ref and I, ignoring the borders. '''
  h, w = I.shape[:2]
  inner = (slice(margin_rows, h - margin_rows), slice(margin_cols, w - margin_cols))
  np.testing.assert_allclose(I[inner], ref[inner], rtol=0, atol=atol)


def sobel_reference(I, dx, dy):
  ''' cv::Sobel(I, CV_64F, dx, dy, ksize=5) inner values. '''
  smooth = np.array([1., 4., 6., 4., 1.])
  derivative = np.array([-1., -2., 0., 2., 1.])
  kernel = np.outer(derivative if dy else smooth, derivative if dx else smooth)
  return correlate(I, kernel)


def gaussian_blur_reference(I, size, sigma):
  ''' cv::GaussianBlur(I, Size(size, size), sigma) inner values. '''
  x = np.arange(size) - (size - 1) / 2.
  g = np.exp(-x**2 / (2 * sigma**2))
  g /= g.sum()
  return correlate(I, np.outer(g, g))


@pytest.fixture(scope='module')
def klimt_gray():
  I = ImageGray()
  ImageIo.read(I, os.path.join(DATASET_PATH, 'Klimt', 'Klimt.pgm'))
  return I


@pytest.fixture(scope='module')
def klimt_rgba():
  I = ImageRGBa()
  ImageIo.read(I, os.path.join(DATASET_PATH, 'Klimt', 'Klimt.ppm'))
  return I


def small_image():
  return ImageGray(np.arange(36, dtype=np.uint8).reshape(6, 6))


def check_filter(I, kernel, convolve):
  If = ImageDouble()
  ImageFilter.filter(I, If, ArrayDouble2D(kernel), convolve)
  assert If.getRows() == I.getRows() and If.getCols() == I.getCols()
  ref = correlate(I.numpy(), kernel, convolve)
  check_results(ref, If.numpy(), kernel.shape[0] // 2, kernel.shape[1] // 2, np.finfo(np.float64).eps)


@pytest.mark.parametrize('convolve', [False, True], ids=['correlation', 'convolution'])
@pytest.mark.parametrize('kernel', KERNELS, ids=KERNEL_IDS)
def test_filter_small_image(kernel, convolve):
  check_filter(small_image(), kernel, convolve)


@requires_dataset
@pytest.mark.parametrize('convolve', [False, True], ids=['correlation', 'convolution'])
@pytest.mark.parametrize('kernel', KERNELS, ids=KERNEL_IDS)
def test_filter_dataset(klimt_gray, kernel, convolve):
  check_filter(klimt_gray, kernel, convolve)


def sobel_x_kernel():
  ''' getSobelKernelX() returns the kernel to use for a correlation, flip it to use a convolution. '''
  kernel = ArrayDouble2D()
  ImageFilter.getSobelKernelX(kernel, 5)
  return ArrayDouble2D(np.ascontiguousarray(kernel.numpy()[:, ::-1]))


@requires_dataset
def test_sobel(klimt_gray):
  kernel = sobel_x_kernel()
  I_sobel_x = ImageDouble()
  ImageFilter.filter(klimt_gray, I_sobel_x, kernel, True)
  check_results(sobel_reference(klimt_gray.numpy(), 1, 0), I_sobel_x.numpy(), 2, 2, 1e-9)


@requires_dataset
def test_sobel_u_v(klimt_gray):
  kernel = sobel_x_kernel()
  I_double = ImageDouble(klimt_gray.numpy().astype(np.float64))
  Iu, Iv = ImageDouble(), ImageDouble()
  ImageFilter.filter(I_double, Iu, Iv, kernel, True)
  check_results(sobel_reference(klimt_gray.numpy(), 1, 0), Iu.numpy(), 2, 2, 1e-9)
  check_results(sobel_reference(klimt_gray.numpy(), 0, 1), Iv.numpy(), 2, 2, 1e-9)


@requires_dataset
def test_separable_filter(klimt_gray):
  # Sobel X 5x5 kernel decomposed as two separable kernels, applied as a convolution
  kernel_sep_x = ColVector([1., 2., 0., -2., -1.])
  kernel_sep_y = ColVector([1., 4., 6., 4., 1.])
  I_sep_filtered = ImageDouble()
  ImageFilter.separableFilter(klimt_gray, I_sep_filtered, kernel_sep_x, kernel_sep_y)
  assert I_sep_filtered.getRows() == klimt_gray.getRows() and I_sep_filtered.getCols() == klimt_gray.getCols()
  check_results(sobel_reference(klimt_gray.numpy(), 1, 0), I_sep_filtered.numpy(), 2, 2, 1e-9)


def test_median_gray():
  I_median = ImageGray(np.arange(9, dtype=np.uint8).reshape(3, 3))
  assert ImageFilter.median(I_median) == pytest.approx(4.)


def test_median_rgba():
  values = np.arange(9, dtype=np.uint8).reshape(3, 3)
  rgba = np.zeros((3, 3, 4), dtype=np.uint8)
  rgba[..., 0] = values
  rgba[..., 1] = 2 * values
  rgba[..., 2] = 3 * values
  medians = ImageFilter.median(ImageRGBa(rgba))
  assert medians == pytest.approx([4., 8., 12.])


@requires_dataset
def test_gaussian_blur_gray(klimt_gray):
  size, sigma = 7, 3.
  I_blur = ImageDouble()
  ImageFilter.gaussianBlur(klimt_gray, I_blur, size, sigma)
  assert I_blur.getRows() == klimt_gray.getRows() and I_blur.getCols() == klimt_gray.getCols()
  ref = gaussian_blur_reference(klimt_gray.numpy(), size, sigma)
  # Output is stored as double: ViSP and OpenCV Gaussian kernels are the same
  check_results(ref, I_blur.numpy(), 3, 3, 1e-6)


@requires_dataset
def test_gaussian_blur_rgba(klimt_rgba):
  size, sigma = 7, 3.
  I_blur = ImageRGBa()
  ImageFilter.gaussianBlur(klimt_rgba, I_blur, size, sigma)
  assert I_blur.getRows() == klimt_rgba.getRows() and I_blur.getCols() == klimt_rgba.getCols()
  # Output is stored as unsigned char, use the same threshold as the C++ test
  for channel in range(3):  # R, G, B
    ref = gaussian_blur_reference(klimt_rgba.numpy()[..., channel], size, sigma)
    check_results(ref, I_blur.numpy()[..., channel].astype(np.float64), 3, 3, 3.)
