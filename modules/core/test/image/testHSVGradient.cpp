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
void gradientFilter(const vpImage<unsigned char> &I, const vpImage<double> &filter, vpImage<double> &GIy)
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

void gradientFilter(const vpImage<unsigned char> &I, const vpImage<double> &filterX, const vpImage<double> &filterY, vpImage<double> &GIx, vpImage<double> &GIy)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  GIy.resize(nbRows, nbCols, 0.);
  gradientFilter(I, filterX, GIx);
  gradientFilter(I, filterY, GIy);
}

static bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c)
{
  bool computeVal = true;
  if (p_mask != nullptr) {
    computeVal = (*p_mask)[r][c];
  }
  return computeVal;
}

template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterX(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, const vpImage<bool> *p_mask, const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  std::vector<FilterType> filter(3);
  FilterType scale;
  std::string name;
  switch (type) {
  case vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING:
    filter = { 1., 2., 1. };
    scale = 8.;
    name = "Sobel";
    break;
  case vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING:
    filter = { 3., 10., 3. };
    scale = 32.;
    name = "Scharr";
    break;
  default:
    throw(vpException(vpException::badValue, "Wrong type of filtering"));
  }
  // std::cout << "Using " << name << " filter" << std::endl;
  for (unsigned char i = 0; i < 3; ++i) {
    filter[i] = filter[i] / scale;
  }

  auto checkBooleanPatch = [](const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c, const unsigned int &h, const unsigned int &w)
    {
      if (!p_mask) {
        return true;
      }
      bool hasToCompute = (*p_mask)[r][c];

      if (c < w - 1) { // We do not compute gradient on the last column
        hasToCompute |= (*p_mask)[r][c + 1];
        if (r < h - 1) { // We do not compute gradient on the last row
          hasToCompute |= (*p_mask)[r + 1][c + 1];
        }
      }

      if (r < h - 1) { // We do not compute gradient on the last row
        hasToCompute |= (*p_mask)[r + 1][c];
      }

      if (r > 1) { // We do not compute gradient on the first row
        hasToCompute |= (*p_mask)[r - 1][c];
        if (c < w - 1) { // We do not compute gradient on the last column
          hasToCompute |= (*p_mask)[r - 1][c + 1];
        }
      }
      return hasToCompute;
    };

  const unsigned int rStop = nbRows - 1, cStop = nbCols - 1;
  vpImage<double> IabsDiff(nbRows, nbCols);

  // Computation of the rest of the image
  for (unsigned int r = 1; r < rStop; ++r) {
    // Computation for I[r][0]
    if (checkBooleanPatch(p_mask, r, 0, nbRows, nbCols)) {
      IabsDiff[r][0] = I[r][1].V - I[r][0].V;
    }

      // Computation for all the other columns
    for (unsigned int c = 1; c < cStop; ++c) {
      if (checkBooleanPatch(p_mask, r, c, nbRows, nbCols)) {
        // Of the absolute value of the distance
        IabsDiff[r][c] = I[r][c + 1].V - I[r][c].V;
      }
    }
  }

  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 1; c < cStop; ++c) {
      if (checkBooleanMask(p_mask, r, c)) {
        GIx[r][c] = 0.;
        for (int dr = -1; dr <= 1; ++dr) {
          GIx[r][c] += filter[dr + 1] * (IabsDiff[r + dr][c - 1] +  IabsDiff[r + dr][c]);
        }
      }
    }
  }
}

template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterY(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIy, const vpImage<bool> *p_mask, const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  std::vector<FilterType> filter(3);
  FilterType scale;
  switch (type) {
  case vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING:
    filter = { 1., 2., 1. };
    scale = 8.;
    break;
  case vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING:
    filter = { 3., 10., 3. };
    scale = 32.;
    break;
  default:
    throw(vpException(vpException::badValue, "Wrong type of filtering"));
  }
  for (unsigned char i = 0; i < 3; ++i) {
    filter[i] = filter[i] / scale;
  }

  const unsigned int rStop = nbRows - 1, cStop = nbCols - 1;
  vpImage<double> IabsDiff(nbRows, nbCols);

  auto checkBooleanPatch = [](const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c, const unsigned int &h, const unsigned int &w)
    {
      if (!p_mask) {
        return true;
      }

      bool hasToCompute = (*p_mask)[r][c];
      if (c < w - 1) { // We do not compute gradient on the last column
        hasToCompute |= (*p_mask)[r][c + 1];
        if (r < h - 1) { // We do not compute gradient on the last row
          hasToCompute |= (*p_mask)[r + 1][c + 1];
        }
      }

      if (r < h - 1) { // We do not compute gradient on the last row
        hasToCompute |= (*p_mask)[r + 1][c];
      }

      if (c > 1) { // We do not compute gradient on the first column
        hasToCompute |= (*p_mask)[r][c - 1];
        if (r < h - 1) { // We do not compute gradient on the last row
          hasToCompute |= (*p_mask)[r + 1][c - 1];
        }
      }
      return hasToCompute;
    };

  // Computation for the first row
  for (unsigned int c = 0; c < nbCols; ++c) {
    if (checkBooleanPatch(p_mask, 0, c, nbRows, nbCols)) {
      IabsDiff[0][c] = I[1][c].V - I[0][c].V;
    }
  }

  // Computation for the rest of the image of d and sign
  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      // Of the absolute value of the distance
      if (checkBooleanPatch(p_mask, r, c, nbRows, nbCols)) {
        IabsDiff[r][c] = I[r + 1][c].V - I[r][c].V;
      }
    }
  }

  // Computation of the gradient
  for (unsigned int r = 1; r < rStop; ++r) {
    for (unsigned int c = 1; c < cStop; ++c) {
      if (checkBooleanMask(p_mask, r, c)) {
        GIy[r][c] = 0.;
        for (int dc = -1; dc <= 1; ++dc) {
          GIy[r][c] += filter[dc + 1] * (IabsDiff[r - 1][c + dc] + IabsDiff[r][c + dc]);
        }
      }
    }
  }
}

template <typename ArithmeticType, bool useFullScale>
void gradientFilter(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<double> &GIx, vpImage<double> &GIy, const vpImage<bool> *p_mask, const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  GIy.resize(nbRows, nbCols, 0.);
  gradientFilterX(I, GIx, p_mask, type);
  gradientFilterY(I, GIy, p_mask, type);
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

  std::vector<vpImageFilter::vpCannyFilteringAndGradientType> types = {
    vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING,
    vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING
  };

  std::vector<int> nbThreads = { 1, 2 };

  vpImage<bool> *p_mask = nullptr;
  for (unsigned int i = 0; i < 2; ++i) {
    if (i == 1) {
      p_mask = &dataset.m_Imask;
    }
    for (auto type: types) {
      for (auto nbThread: nbThreads) {
        for (auto input: dataset.m_hsvUCtrue) {
          vpImageFilter::gradientFilter(input.second.m_I, GIx, GIy, nbThread, p_mask, type);
          gradientFilter(input.second.m_I, GIx_ref, GIy_ref, p_mask, type);
          bool isSuccessGIx = vpHSVTests::areAlmostEqual(GIx, "GIx", GIx_ref, "GIx_ref");
          bool isSuccessGIy = vpHSVTests::areAlmostEqual(GIy, "GIy", GIy_ref, "GIy_ref");
          isSuccess = isSuccess && isSuccessGIx && isSuccessGIy;
          if (!isSuccessGIx) {
            std::cerr << "ERROR: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << " along X on HSV<uchar, true> failed ! " << std::endl;
          }
          if (!isSuccessGIy) {
            std::cerr << "ERROR: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << " along Y on HSV<uchar, true> failed ! " << std::endl;
          }
          if (!(isSuccessGIx && isSuccessGIy)) {
            std::cout << "Type:  " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << std::endl;
            std::cout << "nbThread:  " << nbThread << std::endl;
            std::cout << "mask ? :  " << (p_mask ? std::string("true") : std::string("false")) << std::endl;
            vpHSVTests::print(input.second.m_I, input.first);
            vpHSVTests::print(GIx, "GIx");
            vpHSVTests::print(GIy, "GIy");
            vpHSVTests::print(GIx_ref, "GIx_ref");
            vpHSVTests::print(GIy_ref, "GIy_ref");
          }
        }

        for (auto input: dataset.m_hsvUCfalse) {
          vpImageFilter::gradientFilter(input.second.m_I, GIx, GIy, nbThread, p_mask, type);
          gradientFilter(input.second.m_I, GIx_ref, GIy_ref, p_mask, type);
          bool isSuccessGIx = vpHSVTests::areAlmostEqual(GIx, "GIx", GIx_ref, "GIx_ref");
          bool isSuccessGIy = vpHSVTests::areAlmostEqual(GIy, "GIy", GIy_ref, "GIy_ref");
          isSuccess = isSuccess && isSuccessGIx && isSuccessGIy;
          if (!isSuccessGIx) {
            std::cerr << "ERROR: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << " along X on HSV<uchar, false> failed ! " << std::endl;
          }
          if (!isSuccessGIy) {
            std::cerr << "ERROR: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << " along Y on HSV<uchar, false> failed ! " << std::endl;
          }
          if (!(isSuccessGIx && isSuccessGIy)) {
            std::cout << "Type:  " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << std::endl;
            std::cout << "nbThread:  " << nbThread << std::endl;
            std::cout << "mask ? :  " << (p_mask ? std::string("true") : std::string("false")) << std::endl;
            vpHSVTests::print(input.second.m_I, input.first);
            vpHSVTests::print(GIx, "GIx");
            vpHSVTests::print(GIy, "GIy");
            vpHSVTests::print(GIx_ref, "GIx_ref");
            vpHSVTests::print(GIy_ref, "GIy_ref");
          }
        }

        for (auto input: dataset.m_hsvDouble) {
          vpImageFilter::gradientFilter(input.second.m_I, GIx, GIy, nbThread, p_mask, type);
          gradientFilter(input.second.m_I, GIx_ref, GIy_ref, p_mask, type);
          bool isSuccessGIx = vpHSVTests::areAlmostEqual(GIx, "GIx", GIx_ref, "GIx_ref");
          bool isSuccessGIy = vpHSVTests::areAlmostEqual(GIy, "GIy", GIy_ref, "GIy_ref");
          isSuccess = isSuccess && isSuccessGIx && isSuccessGIy;
          if (!isSuccessGIx) {
            std::cerr << "ERROR: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << " along X on HSV<double> failed ! " << std::endl;
          }
          if (!isSuccessGIy) {
            std::cerr << "ERROR: " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << " along Y on HSV<double> failed ! " << std::endl;
          }
          if (!(isSuccessGIx && isSuccessGIy)) {
            std::cout << "Type:  " << vpImageFilter::vpCannyFiltAndGradTypeToStr(type) << std::endl;
            std::cout << "nbThread:  " << nbThread << std::endl;
            std::cout << "mask ? :  " << (p_mask ? std::string("true") : std::string("false")) << std::endl;
            vpHSVTests::print(input.second.m_I, input.first);
            vpHSVTests::print(GIx, "GIx");
            vpHSVTests::print(GIy, "GIy");
            vpHSVTests::print(GIx_ref, "GIx_ref");
            vpHSVTests::print(GIy_ref, "GIy_ref");
          }
        }
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
