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
 * Test vpImageFilter::gaussianBlur() new implementation and compare it to the old one.
 */
/*!
  \example testImageFilterHSVOldVSNew.cpp

  \brief Test vpImageFilter::gaussianBlur() new implementation and compare it to the old one.
*/

#include <iostream>
#include <limits>
#include <type_traits>

#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>

#include "hsvUtils.h"

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
void prewittFilter(const vpImage<unsigned char> &I, const vpImage<double> &filter, vpImage<double> &GIy)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  for (unsigned int r = 1; r < nbRows - 1; ++r) {
    for (unsigned int c = 1; c < nbCols - 1; ++c) {
      for (unsigned int i = 0; i <= 2; ++i) {
        for (unsigned int j = 0; j <= 2; ++j) {
          GIy[r][c] += filter[i][j] * I[r-1 + i][c-1 + j];
        }
      }
    }
  }
}

void prewittFilter(const vpImage<unsigned char> &I, const vpImage<double> &filterX, const vpImage<double> &filterY, vpImage<double> &GIx, vpImage<double> &GIy)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  GIy.resize(nbRows, nbCols, 0.);
  prewittFilter(I, filterX, GIx);
  prewittFilter(I, filterY, GIy);
}

template <typename ArithmeticType, bool useFullScale>
void prewittFilterX(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<double> &GIx)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  const unsigned int rStop = nbRows - 1, cStop = nbCols - 1;
  vpImage<double> Isign(nbRows, nbCols), IabsDiff(nbRows, nbCols);
  // Computation for I[0][0]
  if (vpColVector::dotProd((I[0][1] - I[0][0]), I[0][0].toColVector()) < 0.) {
    Isign[0][0] = -1.;
  }
  else {
    Isign[0][0] = 1.;
  }
  // Computation for the rest of the first row
  for (unsigned int c = 1; c < cStop; ++c) {
    if (vpColVector::dotProd((I[0][c + 1] - I[0][c]), (I[0][c] - I[0][c - 1])) < 0.) {
      // Inverting sign when cosine distance is negative
      Isign[0][c] = -1. * Isign[0][c - 1];
    }
    else {
      Isign[0][c] = Isign[0][c - 1];
    }
  }
  for (unsigned int r = 1; r < rStop; ++r) {
    // Computation for I[r][0]
    if (vpColVector::dotProd((I[r][1] - I[r][0]), I[r][0].toColVector()) < 0.) {
      Isign[r][0] = -1.;
    }
    else {
      Isign[r][0] = 1.;
    }
    IabsDiff[r][0] = vpHSV<ArithmeticType, useFullScale>::template squaredMahalanobisDistance<double>(I[r][0], I[r][1]);

    // Computation for all the other columns
    for (unsigned int c = 1; c < cStop; ++c) {
      // Of the absolute value of the distance
      IabsDiff[r][c] = vpHSV<ArithmeticType, useFullScale>::template squaredMahalanobisDistance<double>(I[r][c], I[r][c + 1]);
      // Of the sign
      if (vpColVector::dotProd((I[r][c + 1] - I[r][c]), (I[r][c] - I[r][c - 1])) < 0.) {
        // Inverting sign when cosine distance is negative
        Isign[r][c] = -1. * Isign[r][c - 1];
      }
      else {
        Isign[r][c] = Isign[r][c - 1];
      }
    }
  }

  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 1; c < cStop; ++c) {
      GIx[r][c] = 0.;
      for (int dr = -1; dr <= 1; ++dr) {
        GIx[r][c] += Isign[r + dr][c - 1] * IabsDiff[r + dr][c - 1] + Isign[r + dr][c] * IabsDiff[r + dr][c];
      }
    }
  }
}

template <typename ArithmeticType, bool useFullScale>
void prewittFilterY(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<double> &GIy)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  const unsigned int rStop = nbRows - 1, cStop = nbCols - 1;
  vpImage<double> Isign(nbRows, nbCols), IabsDiff(nbRows, nbCols);
  // Computation for the first row
  for (unsigned int c = 0; c < nbCols; ++c) {
    IabsDiff[0][c] = vpHSV<ArithmeticType, useFullScale>::template squaredMahalanobisDistance<double>(I[0][c], I[1][c]);
    if (vpColVector::dotProd((I[1][c] - I[0][c]), I[0][c].toColVector()) < 0.) {
      // Inverting sign when cosine distance is negative
      Isign[0][c] = -1.;
    }
    else {
      Isign[0][c] = 1.;
    }
  }
  for (unsigned int r = 1; r < rStop; ++r) {
    // Computation for all the other columns
    for (unsigned int c = 0; c < nbCols; ++c) {
      // Of the absolute value of the distance
      IabsDiff[r][c] = vpHSV<ArithmeticType, useFullScale>::template squaredMahalanobisDistance<double>(I[r][c], I[r + 1][c]);
      // Of the sign
      if (vpColVector::dotProd((I[r +1][c] - I[r][c]), (I[r][c] - I[r - 1][c])) < 0.) {
        // Inverting sign when cosine distance is negative
        Isign[r][c] = -1. * Isign[r - 1][c];
      }
      else {
        Isign[r][c] = Isign[r - 1][c];
      }
    }
  }

  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 1; c < cStop; ++c) {
      GIy[r][c] = 0.;
      for (int dc = -1; dc <= 1; ++dc) {
        GIy[r][c] += Isign[r - 1][c + dc] * IabsDiff[r - 1][c + dc] + Isign[r][c + dc] * IabsDiff[r][c + dc];
      }
    }
  }
}

template <typename ArithmeticType, bool useFullScale>
void prewittFilter(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<double> &GIx, vpImage<double> &GIy)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  GIy.resize(nbRows, nbCols, 0.);
  prewittFilterX(I, GIx);
  prewittFilterY(I, GIy);
}

int main()
{
  bool isSuccess = true;

  // Inputs
  vpHSVTests::vpInputDataset dataset;
  bool useSobel = false;
  vpImage<double> filterX(3, 3);
  filterX[0][0] = -1.; filterX[0][1] = 0.; filterX[0][2] = 1.;
  filterX[1][0] = (useSobel ? -2. : -1.); filterX[1][1] = 0.; filterX[1][2] = (useSobel ? 2. : 1.);
  filterX[2][0] = -1.; filterX[2][1] = 0.; filterX[2][2] = 1.;

  vpImage<double> filterY(3, 3);
  filterY[0][0] = -1.; filterY[0][1] = (useSobel ? -2. : -1.); filterY[0][2] = -1.;
  filterY[1][0] = 0.; filterY[1][1] = 0.; filterY[1][2] = 0.;
  filterY[2][0] = 1.; filterY[2][1] = (useSobel ? 2. : 1.); filterY[2][2] = 1.;

  // Outputs
  vpImage<double> GIx, GIx_ref, GIy, GIy_ref;

  for (unsigned int size = 3; (size < 7) && isSuccess; size += 2) {
    for (auto input: dataset.m_hsvUCtrue) {
      vpHSVTests::print(input.second.m_I, input.first);
      vpImageFilter::prewittFilter(input.second.m_I, GIx, GIy);
      vpHSVTests::print(GIx, "GIx");
      vpHSVTests::print(GIy, "GIy");
      prewittFilter(input.second.m_I, GIx_ref, GIy_ref);
      // vpHSVTests::print(GIx_ref, "GIx_ref");
      // vpHSVTests::print(GIy_ref, "GIy_ref");
      bool hsvucSuccessGIx = vpHSVTests::areAlmostEqual(GIx, GIx_ref);
      bool hsvucSuccessGIy = vpHSVTests::areAlmostEqual(GIy, GIy_ref);
      isSuccess = isSuccess && hsvucSuccessGIx && hsvucSuccessGIy;
      if (!hsvucSuccessGIx) {
        std::cerr << "ERROR: Prewitt along X on HSV<uchar, true> failed ! " << std::endl;
      }
      if (!hsvucSuccessGIy) {
        std::cerr << "ERROR: Prewitt along Y on HSV<uchar, true> failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvUCfalse) {
      vpHSVTests::print(input.second.m_I, input.first);
      vpImageFilter::prewittFilter(input.second.m_I, GIx, GIy);
      vpHSVTests::print(GIx, "GIx");
      vpHSVTests::print(GIy, "GIy");
      prewittFilter(input.second.m_I, GIx_ref, GIy_ref);
      // vpHSVTests::print(GIx_ref, "GIx_ref");
      // vpHSVTests::print(GIy_ref, "GIy_ref");
      bool hsvucSuccessGIx = vpHSVTests::areAlmostEqual(GIx, GIx_ref);
      bool hsvucSuccessGIy = vpHSVTests::areAlmostEqual(GIy, GIy_ref);
      isSuccess = isSuccess && hsvucSuccessGIx && hsvucSuccessGIy;
      if (!hsvucSuccessGIx) {
        std::cerr << "ERROR: Prewitt along X on HSV<uchar, false> failed ! " << std::endl;
      }
      if (!hsvucSuccessGIy) {
        std::cerr << "ERROR: Prewitt along Y on HSV<uchar, false> failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvDouble) {
      vpHSVTests::print(input.second.m_I, input.first);
      vpImageFilter::prewittFilter(input.second.m_I, GIx, GIy);
      vpHSVTests::print(GIx, "GIx");
      vpHSVTests::print(GIy, "GIy");
      prewittFilter(input.second.m_I, GIx_ref, GIy_ref);
      // vpHSVTests::print(GIx_ref, "GIx_ref");
      // vpHSVTests::print(GIy_ref, "GIy_ref");
      bool hsvucSuccessGIx = vpHSVTests::areAlmostEqual(GIx, GIx_ref);
      bool hsvucSuccessGIy = vpHSVTests::areAlmostEqual(GIy, GIy_ref);
      isSuccess = isSuccess && hsvucSuccessGIx && hsvucSuccessGIy;
      if (!hsvucSuccessGIx) {
        std::cerr << "ERROR: Prewitt along X on HSV<double> failed ! " << std::endl;
      }
      if (!hsvucSuccessGIy) {
        std::cerr << "ERROR: Prewitt along Y on HSV<double> failed ! " << std::endl;
      }
    }
  }

  if (isSuccess) {
    std::cout << "All tests were successful !" << std::endl;
    return EXIT_SUCCESS;
  }
  std::cerr << "ERROR: Something went wrong !" << std::endl;
  return EXIT_FAILURE;
}
#else
int main()
{
  std::cout << "vpHSV class is not available, please use CXX 11 standard" << std::endl;
  return EXIT_SUCCESS;
}
#endif
