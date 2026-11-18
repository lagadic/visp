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
# ViSP Python bindings test for visp.core.GaussianFilter and visp.core.ImageFilter.gaussianBlur().
# Python port of modules/core/test/image-with-dataset/perfGaussianFilter.cpp
#
# By default, only checks that the filters run and give consistent results on the dataset image.
# Benchmarks are run when the VISP_PYTHON_BENCHMARK environment variable is set, e.g.:
#   VISP_PYTHON_BENCHMARK=1 python -m pytest -s test_image_filter_perf_gaussian.py
#
#############################################################################

import os
import timeit

import numpy as np
import pytest

from visp.core import GaussianFilter, ImageDouble, ImageGray, ImageRGBa, IoTools
from visp.core import ImageFilter

try:
  from visp.io import ImageIo
except ImportError:
  ImageIo = None

IMAGE_PATH = os.path.join(IoTools.getViSPImagesDataPath(), 'faces', '1280px-Solvay_conference_1927.png')

pytestmark = pytest.mark.skipif(ImageIo is None or not os.path.exists(IMAGE_PATH),
                                reason='ViSP dataset is not available, set VISP_INPUT_IMAGE_PATH environment variable')

requires_benchmark = pytest.mark.skipif(not os.environ.get('VISP_PYTHON_BENCHMARK'),
                                        reason='Set VISP_PYTHON_BENCHMARK environment variable to run benchmarks')

SIGMA = 5.
KERNEL_SIZE = 7


def benchmark(name, fn, repeat=5, number=10):
  times = timeit.repeat(fn, repeat=repeat, number=number)
  best_ms = 1000. * min(times) / number
  print(f'\nBenchmark {name}: {best_ms:.3f} ms')
  return best_ms


@pytest.fixture(scope='module')
def image_gray():
  I = ImageGray()
  ImageIo.read(I, IMAGE_PATH)
  return I


@pytest.fixture(scope='module')
def image_rgba():
  I = ImageRGBa()
  ImageIo.read(I, IMAGE_PATH)
  return I


def check_blurred(I, I_blur):
  assert I_blur.getRows() == I.getRows() and I_blur.getCols() == I.getCols()
  src = I.numpy().astype(np.float64)
  dst = I_blur.numpy().astype(np.float64)
  if src.ndim == 3:  # Alpha channel is not considered
    src, dst = src[..., :3], dst[..., :3]
  # Blurring keeps the mean intensity and smooths the image
  assert dst.mean() == pytest.approx(src.mean(), abs=2.)
  assert np.abs(np.diff(dst, axis=1)).mean() < np.abs(np.diff(src, axis=1)).mean()


def test_gaussian_filter_gray(image_gray):
  gaussian_filter = GaussianFilter(image_gray.getWidth(), image_gray.getHeight(), SIGMA)
  I_blur = ImageGray()
  gaussian_filter.apply(image_gray, I_blur)
  check_blurred(image_gray, I_blur)


@pytest.mark.parametrize('deinterleave', [False, True], ids=['interleaved', 'deinterleave'])
def test_gaussian_filter_rgba(image_rgba, deinterleave):
  gaussian_filter = GaussianFilter(image_rgba.getWidth(), image_rgba.getHeight(), SIGMA, deinterleave)
  I_blur = ImageRGBa()
  gaussian_filter.apply(image_rgba, I_blur)
  check_blurred(image_rgba, I_blur)


def test_gaussian_filter_deinterleave_consistency(image_rgba):
  # Deinterleaving the channels is an optimization, results are expected to be the same
  results = []
  for deinterleave in [False, True]:
    I_blur = ImageRGBa()
    GaussianFilter(image_rgba.getWidth(), image_rgba.getHeight(), SIGMA, deinterleave).apply(image_rgba, I_blur)
    results.append(I_blur.numpy()[..., :3].astype(np.int16))
  assert np.abs(results[0] - results[1]).max() <= 1


def test_gaussian_blur_gray(image_gray):
  I_blur = ImageDouble()
  ImageFilter.gaussianBlur(image_gray, I_blur, KERNEL_SIZE, SIGMA)
  check_blurred(image_gray, I_blur)


def test_gaussian_blur_rgba(image_rgba):
  I_blur = ImageRGBa()
  ImageFilter.gaussianBlur(image_rgba, I_blur, KERNEL_SIZE, SIGMA)
  check_blurred(image_rgba, I_blur)


@requires_benchmark
def test_benchmark_gaussian_filter(image_gray, image_rgba):
  I_blur_gray, I_blur_rgba = ImageGray(), ImageRGBa()
  gaussian_filter_gray = GaussianFilter(image_gray.getWidth(), image_gray.getHeight(), SIGMA)
  benchmark('GaussianFilter uchar', lambda: gaussian_filter_gray.apply(image_gray, I_blur_gray))
  gaussian_filter_rgba = GaussianFilter(image_rgba.getWidth(), image_rgba.getHeight(), SIGMA)
  benchmark('GaussianFilter RGBa', lambda: gaussian_filter_rgba.apply(image_rgba, I_blur_rgba))
  gaussian_filter_rgba = GaussianFilter(image_rgba.getWidth(), image_rgba.getHeight(), SIGMA, True)
  benchmark('GaussianFilter RGBa + deinterleave', lambda: gaussian_filter_rgba.apply(image_rgba, I_blur_rgba))


@requires_benchmark
def test_benchmark_gaussian_blur(image_gray, image_rgba):
  I_blur_double, I_blur_rgba = ImageDouble(), ImageRGBa()
  benchmark('ImageFilter.gaussianBlur uchar',
            lambda: ImageFilter.gaussianBlur(image_gray, I_blur_double, KERNEL_SIZE, SIGMA))
  benchmark('ImageFilter.gaussianBlur RGBa',
            lambda: ImageFilter.gaussianBlur(image_rgba, I_blur_rgba, KERNEL_SIZE, SIGMA))
