/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Image XY filtering.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageFilter.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
double vpImageFilter::filterXR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;
  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][c - i].R);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double vpImageFilter::filterXG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][c - i].G);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double vpImageFilter::filterXB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][c - i].B);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double vpImageFilter::filterXLeftBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][c - i].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][i - c].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double vpImageFilter::filterXLeftBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][c - i].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][i - c].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double vpImageFilter::filterXLeftBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][c - i].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][i - c].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double vpImageFilter::filterXRightBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][c - i].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][((2 * width) - c) - i - 1].R + I[r][c - i].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double vpImageFilter::filterXRightBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][c - i].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][((2 * width) - c) - i - 1].G + I[r][c - i].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double vpImageFilter::filterXRightBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][c - i].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][(2 * width) - c - i - 1].B + I[r][c - i].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double vpImageFilter::filterYR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].R + I[r - i][c].R);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double vpImageFilter::filterYG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].G + I[r - i][c].G);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double vpImageFilter::filterYB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].B + I[r - i][c].B);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double vpImageFilter::filterYTopBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].R + I[r - i][c].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].R + I[i - r][c].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double vpImageFilter::filterYTopBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].G + I[r - i][c].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].G + I[i - r][c].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double vpImageFilter::filterYTopBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].B + I[r - i][c].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].B + I[i - r][c].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double vpImageFilter::filterYBottomBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].R + I[r - i][c].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[((2 * height) - r) - i - 1][c].R + I[r - i][c].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double vpImageFilter::filterYBottomBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].G + I[r - i][c].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[((2 * height) - r) - i - 1][c].G + I[r - i][c].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double vpImageFilter::filterYBottomBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].B + I[r - i][c].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[((2 * height) - r) - i - 1][c].B + I[r - i][c].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

#endif

END_VISP_NAMESPACE
