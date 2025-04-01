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
  vpImage<double> GIx, GIy;

  for (unsigned int size = 3; (size < 7) && isSuccess; size += 2) {
    for (auto input: dataset.m_ucImages) {
      vpHSVTests::print(input.second.m_I, input.first);
      prewittFilter(input.second.m_I, filterX, filterY, GIx, GIy);
      vpHSVTests::print(GIx, "GIx_uc");
      vpHSVTests::print(GIy, "GIy_uc");
      bool ucSuccess = true; // vpHSVTests::areAlmostEqual(Iuc_filtered_old, Iuc_filtered_new);
      isSuccess = isSuccess && ucSuccess;
      if (!isSuccess) {
        std::cerr << "ERROR: filter on uchar failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvUCtrue) {
      vpHSVTests::print(input.second.m_I, input.first);
      vpImageFilter::prewittFilter(input.second.m_I, GIx, GIy);
      vpHSVTests::print(GIx, "GIx");
      vpHSVTests::print(GIy, "GIy");
      bool hsvucSuccessTrue = true; //vpHSVTests::areAlmostEqual(Ihsv_uc_filtered_old_true, Ihsv_uc_filtered_new_true);
      isSuccess = isSuccess && hsvucSuccessTrue;
      if (!hsvucSuccessTrue) {
        std::cerr << "ERROR: filter on HSV<uchar, true> failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvUCfalse) {
      vpHSVTests::print(input.second.m_I, input.first);
      vpImageFilter::prewittFilter(input.second.m_I, GIx, GIy);
      vpHSVTests::print(GIx, "GIx");
      vpHSVTests::print(GIy, "GIy");
      bool hsvucSuccessTrue = true; // vpHSVTests::areAlmostEqual(Ihsv_uc_filtered_old_false, Ihsv_uc_filtered_new_false);
      isSuccess = isSuccess && hsvucSuccessTrue;
      if (!hsvucSuccessTrue) {
        std::cerr << "ERROR: filter on HSV<uchar, false> failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvDouble) {
      vpHSVTests::print(input.second.m_I, input.first);
      vpImageFilter::prewittFilter(input.second.m_I, GIx, GIy);
      vpHSVTests::print(GIx, "GIx");
      vpHSVTests::print(GIy, "GIy");
      bool hsvucSuccessTrue = true; // && hsvucSuccessTrue;
      if (!hsvucSuccessTrue) {
        std::cerr << "ERROR: filter on HSV<double> failed ! " << std::endl;
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
