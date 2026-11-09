/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
 * Implementation of the templated methods of vpImageFilter.
 */

/*!
 * \file vpImageFilter_impl.h
 * \brief Implementation of the templated methods of vpImageFilter.
 */

#ifndef VP_IMAGE_FILTER_IMPL_H
#define VP_IMAGE_FILTER_IMPL_H

#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColorGetter.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#if (VISP_HAVE_OPENCV_VERSION < 0x050000)
#include <opencv2/imgproc/imgproc_c.h>
#endif
#endif

#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdocumentation"
#endif

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace vpImageFilter
{
namespace detail
{
template <typename FilterType>
FilterType filter(const vpImage<FilterType> &I, const vpArray2D<FilterType> &M, unsigned int row, unsigned int col)
{
  const unsigned int size_y = M.getRows(), size_x = M.getCols();
  const unsigned int half_size_y = size_y / 2, half_size_x = size_x / 2;
  FilterType corr = 0;

  for (unsigned int a = 0; a < size_y; ++a) {
    for (unsigned int b = 0; b < size_x; ++b) {
      FilterType val = static_cast<FilterType>(I[row - half_size_y + a][col - half_size_x + b]); // Correlation
      corr += M[a][b] * val;
    }
  }
  return corr;
}

template <class T> double gaussianFilter(const vpImage<T> &fr, unsigned int r, unsigned int c)
{
  const int val2 = 2;
  return ((15.0 * fr[r][c]) +
          (12.0 * (fr[r - 1][c] + fr[r][c - 1] + fr[r + 1][c] + fr[r][c + 1])) +
          (9.0 * (fr[r - 1][c - 1] + fr[r + 1][c - 1] + fr[r - 1][c + 1] + fr[r + 1][c + 1])) +
          (5.0 * (fr[r - val2][c] + fr[r][c - val2] + fr[r + val2][c] + fr[r][c + val2])) +
          (4.0 * (fr[r - val2][c + 1] + fr[r - val2][c - 1] + fr[r - 1][c - val2] + fr[r + 1][c - val2] + fr[r + val2][c - 1] +
                  fr[r + val2][c + 1] + fr[r - 1][c + val2] + fr[r + 1][c + val2])) +
          (2.0 * (fr[r - val2][c - val2] + fr[r + val2][c - val2] + fr[r - val2][c + val2] + fr[r + val2][c + val2]))) / 159.0;
}

template<typename ImageType>
void resizeAndInitializeIfNeeded(const vpImage<bool> *p_mask, const unsigned int height, const unsigned int width, vpImage<ImageType> &I)
{
  if (p_mask == nullptr) {
    // Just need to resize the output image, values will be computed and overwrite what is inside the image
    I.resize(height, width);
  }
  else {
    // Need to reset the image because some points will not be computed
    I.resize(height, width, static_cast<ImageType>(0));
  }
}

inline bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c)
{
  bool computeVal = true;
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  if (p_mask != nullptr)
#else
  if (p_mask != NULL)
#endif
  {
    computeVal = (*p_mask)[r][c];
  }
  return computeVal;
}

inline bool checkBooleanMask(const vpImage<bool> *p_mask, const int &id)
{
  bool computeVal = true;
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  if (p_mask != nullptr)
#else
  if (p_mask != NULL)
#endif
  {
    computeVal = p_mask->bitmap[id];
  }
  return computeVal;
}

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
  // Helper to apply the scale to the raw values of the filters
template <typename FilterType>
void scaleFilter(vpArray2D<FilterType> &filter, const float &scale)
{
  const unsigned int nbRows = filter.getRows();
  const unsigned int nbCols = filter.getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      filter[r][c] = filter[r][c] * scale;
    }
  }
}
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
inline bool checkBooleanPatch(const vpImage<bool> *p_mask, const int &iter, const int &c,
                       const int &h, const int &w,
                       const bool &isGradientX)
{
  if (!p_mask) {
    return true;
  }
  const int maxIter = (h - 1) * w;
  const int minIter = w;
  bool hasToCompute = p_mask->bitmap[iter];
  if (c < w - 1) { // We do not compute gradient on the last column
    hasToCompute |= p_mask->bitmap[iter + 1]; // I[r][c + 1];
    if (iter < maxIter) { // We do not compute gradient on the last row
      hasToCompute |= p_mask->bitmap[iter + w + 1]; // I[r + 1][c + 1];
    }
  }

  if (iter < maxIter) { // We do not compute gradient on the last row
    hasToCompute |= p_mask->bitmap[iter + w]; // I[r + 1][c];
  }

  if (isGradientX) {
    if (iter > minIter) { // We do not compute gradient on the first row
      hasToCompute |= p_mask->bitmap[iter - w]; // I[r - 1][c];
      if (c < w - 1) { // We do not compute gradient on the last column
        hasToCompute |= p_mask->bitmap[iter - w + 1]; // I[r - 1][c + 1];
      }
    }
  }
  else {
    if (c > 1) { // We do not compute gradient on the first column
      hasToCompute |= p_mask->bitmap[iter - 1]; // Checking mask[r][c - 1]
      if (iter < maxIter) { // We do not compute gradient on the last row
        hasToCompute |= p_mask->bitmap[iter + w - 1]; // Checking mask[r + 1][c - 1]
      }
    }
  }
  return hasToCompute;
}


template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageX(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff
)
{
  const unsigned int nbCols = I.getCols();

  Idiff[0] = static_cast<OutputType>(I.bitmap[1].V - I.bitmap[0].V);

  // Computing the difference and sign for row 1 column 0
  Idiff[nbCols] = static_cast<OutputType>(I.bitmap[nbCols + 1].V - I.bitmap[nbCols].V);

  unsigned int limit = nbCols - 1;
  for (unsigned int iter = 1; iter < limit; ++iter) {
    // Computing the difference and sign for row 0
    OutputType distanceRow0 = static_cast<OutputType>(I.bitmap[iter + 1].V - I.bitmap[iter].V);
    Idiff[iter] = distanceRow0;

    // Computing the difference and sign for row 1
    OutputType distanceRow1 = static_cast<OutputType>(I.bitmap[nbCols + iter + 1].V - I.bitmap[nbCols + iter].V);
    Idiff[nbCols + iter] = distanceRow1;
  }
}

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterXMonothread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const vpImage<bool> *p_mask
)
{
  const int nbRows = static_cast<int>(I.getRows()), nbCols = static_cast<int>(I.getCols());
  const int size = static_cast<int>(I.getSize());
  const int offsetIdiff = nbCols;

  std::vector<OutputType> Idiff(static_cast<size_t>(size));
  initGradientFilterDifferenceImageX(I, Idiff);
  const int resetCounter = nbCols - 1;
  const int stopIter = size - (nbCols + 1);
  int counter = resetCounter, idCol = 0;
  for (int iter = nbCols; iter < stopIter; ++iter) {
    if (counter) {
      // Computing the amplitude of the difference
      OutputType futureDiff = 0.;
      if (checkBooleanPatch(p_mask, iter + offsetIdiff, idCol, nbRows, nbCols, true)) {
        futureDiff = static_cast<OutputType>(I.bitmap[static_cast<size_t>(iter + nbCols +1)].V - I.bitmap[static_cast<size_t>(iter + offsetIdiff)].V);
        Idiff[static_cast<size_t>(iter + offsetIdiff)] = futureDiff;
      }
    }
    if (counter) {
      if ((counter != resetCounter)) {
        if (checkBooleanMask(p_mask, iter)) {
          OutputType gradient = 0.;
          int offset = iter - nbCols; // Looking in the row above first
          for (int i = -1; i <= 1; ++i) {
            // Kind of  +/- (I[r + i][c + 1] - I[r + i][c]) +/- (I[r + i][c] - I[r + i][c - 1])
            gradient += filter[static_cast<size_t>(i + 1)] * (Idiff[static_cast<size_t>(offset)] + Idiff[static_cast<size_t>(offset - 1)]);
            offset += nbCols; // Preparing to look in the next row
          }
          GI.bitmap[iter] = gradient;
        }
      }
      --counter;
    }
    else {
      counter = resetCounter;
    }

    if (idCol < resetCounter) {
      ++idCol;
    }
    else {
      idCol = 0;
    }
  }
}

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageY(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff
)
{
  const unsigned int nbCols = I.getCols();
  // Computing the sign and distance for the first row
  for (unsigned int iter = 0; iter < nbCols; ++iter) {
    OutputType distance = static_cast<OutputType>(I.bitmap[iter + nbCols].V - I.bitmap[iter].V);
    Idiff[iter] = distance;
  }
  // Computing the distance and sign for I[1][0]
  OutputType distance = static_cast<OutputType>(I.bitmap[nbCols + nbCols].V - I.bitmap[nbCols].V);
  Idiff[nbCols] = distance;
}

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterYMonothread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const vpImage<bool> *p_mask
)
{
  const int nbRows = static_cast<int>(I.getRows()), nbCols = static_cast<int>(I.getCols());
  const int size = static_cast<int>(I.getSize());
  const int offsetIdiff = 1;

  std::vector<OutputType> Idiff(static_cast<size_t>(size));
  initGradientFilterDifferenceImageY(I, Idiff);
  const int resetCounter = nbCols - 1;
  const int stopIter = size - (nbCols + 1);
  int counter = resetCounter, iterSign = offsetIdiff;
  for (int iter = nbCols; iter < stopIter; ++iter) {
    // Computing the amplitude of the difference
    OutputType futureDiff = 0.;

    if (checkBooleanPatch(p_mask, iter + offsetIdiff, iterSign, nbRows, nbCols, false)) {
      futureDiff = static_cast<OutputType>(I.bitmap[static_cast<size_t>(iter + nbCols +1)].V - I.bitmap[static_cast<size_t>(iter + offsetIdiff)].V);
      Idiff[static_cast<size_t>(iter + offsetIdiff)] = futureDiff;
    }

    if (counter) {
      if ((counter != resetCounter)) {
        if (checkBooleanMask(p_mask, iter)) {
          OutputType gradient = 0.;
          for (int i = -1; i <= 1; ++i) {
            // Kind of +/- (I[r + 1][c + i] - I[r][c + 1]) +/- (I[r][c + i] - I[r - 1][c + 1])
            gradient += filter[static_cast<size_t>(i + 1)] * (Idiff[static_cast<size_t>(iter + i)] + Idiff[static_cast<size_t>(iter - nbCols + i)]);
          }
          GI.bitmap[iter] = gradient;
        }
      }
      --counter;
    }
    else {
      counter = resetCounter;
    }
    if (iterSign < resetCounter) {
      ++iterSign;
    }
    else {
      iterSign = 0;
    }
  }
}

#ifdef VISP_HAVE_OPENMP

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageX(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff,
  const int &istart, const int &iam
)
{
  const unsigned int nbCols = I.getCols();
  const unsigned int start = static_cast<unsigned int>(istart);

  if (iam > 0) {
    Idiff[0] = static_cast<OutputType>(I.bitmap[start - nbCols + 1].V - I.bitmap[start - nbCols].V);
  }

  // Computing the difference and sign for row 1 column 0, which corresponds to the current row of the image
  Idiff[static_cast<size_t>(nbCols)] = static_cast<OutputType>(I.bitmap[start + 1].V - I.bitmap[start].V);

  const unsigned int nbColsM1 = nbCols - 1;
  for (unsigned int iter = 1; iter < nbColsM1; ++iter) {
    if (iam > 0) {
      // Computing the difference and sign for row 0, which corresponds to the previous row of the image
      OutputType distanceRow0 = static_cast<OutputType>(I.bitmap[start - nbCols + iter + 1].V - I.bitmap[start - nbCols + iter].V);
      Idiff[iter] = distanceRow0;
    }

    // Computing the difference and sign for row 1, which corresponds to the current row of the image
    OutputType distanceRow1 = static_cast<OutputType>(I.bitmap[start + iter + 1].V - I.bitmap[start + iter].V);
    Idiff[nbCols + iter] = distanceRow1;
  }
}

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterXMultithread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const int &maxNbThread, const vpImage<bool> *p_mask)
{
  const int nbRows = static_cast<int>(I.getRows()), nbCols = static_cast<int>(I.getCols());
  const int offsetIdiff = nbCols;
  const int resetCounter = nbCols - 1;
  const int nrows(nbRows - 1);

  int nbThread = maxNbThread;
  if (nbThread < 0) {
    nbThread = omp_get_max_threads();
  }

  if (static_cast<int>(nbRows) < (4 * nbThread)) {
    // Not enough rows to split the image between the threads
    gradientFilterXMonothread(I, GI, filter, p_mask);
    return;
  }

  int iam, nt, irows, rstart, istart, istop;

#pragma omp parallel default(shared) private(iam, nt, irows, rstart, istart, istop) num_threads(nbThread)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    irows = nrows / nt;
    // size of partition
    rstart = irows * iam;
    istart = rstart * nbCols; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      irows = nrows - rstart;
    }
    istop = istart + irows * nbCols;

    std::vector<OutputType> Idiff(static_cast<size_t>((irows + 2) * nbCols));
    std::vector<OutputType> GItemp(static_cast<size_t>(irows * nbCols));
    initGradientFilterDifferenceImageX(I, Idiff, istart, iam);

    int counter = resetCounter, idCol = 0;
    int iterStart = (iam != 0 ? istart : istart + nbCols);
    for (int iter = iterStart; iter < istop; ++iter) {
      if (counter) {
        // Computing the amplitude of the difference
        OutputType futureDiff = 0.;
        if (checkBooleanPatch(p_mask, iter + offsetIdiff, idCol, nbRows, nbCols, true)) {
          futureDiff = static_cast<OutputType>(I.bitmap[static_cast<size_t>(iter + nbCols +1)].V - I.bitmap[static_cast<size_t>(iter + offsetIdiff)].V);
          Idiff[static_cast<size_t>(iter + 2 * offsetIdiff - istart)] = futureDiff;
        }
      }
      if (counter) {
        if ((counter != resetCounter)) {
          if (checkBooleanMask(p_mask, iter)) {
            OutputType gradient = 0.;
            int offset = iter - istart; // Looking in the row above first
            for (int i = -1; i <= 1; ++i) {
              // Kind of  +/- (I[r + i][c + 1] - I[r + i][c]) +/- (I[r + i][c] - I[r + i][c - 1])
              gradient += filter[static_cast<size_t>(i + 1)] * (Idiff[static_cast<size_t>(offset)] + Idiff[static_cast<size_t>(offset - 1)]);
              offset += nbCols; // Preparing to look in the next row
            }
            GItemp[static_cast<size_t>(iter - istart)] = gradient;
          }
        }
        --counter;
      }
      else {
        counter = resetCounter;
      }

      if (idCol < resetCounter) {
        ++idCol;
      }
      else {
        idCol = 0;
      }
    }
#pragma omp critical
    {
      std::memcpy(GI.bitmap + istart, GItemp.data(), GItemp.size() * sizeof(OutputType));
    }
  }
}

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type initGradientFilterDifferenceImageY(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, std::vector<OutputType> &Idiff,
  const int &istart
)
{
  const unsigned int nbCols = I.getCols();
  // Computing the sign and distance for the first row, which corresponds to the row above the beginning of the gradient computation in the thread
  unsigned int idDiff = 0;
  const unsigned int start = static_cast<unsigned int>(istart);
  for (unsigned int iter = start - nbCols; iter < start; ++iter) {
    OutputType distance = static_cast<OutputType>(I.bitmap[iter + nbCols].V - I.bitmap[iter].V);
    Idiff[idDiff] = distance;
    ++idDiff;
  }
  // Computing the distance and sign for I[1][0]
  OutputType distance = static_cast<OutputType>(I.bitmap[nbCols + nbCols].V - I.bitmap[nbCols].V);
  Idiff[nbCols] = distance;
}

template <typename HSVType, bool useFullScale, typename OutputType>
typename std::enable_if<std::is_arithmetic<OutputType>::value, void>::type gradientFilterYMultithread(
  const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<OutputType> &GI, const std::vector<OutputType> &filter,
  const int &maxNbThread, const vpImage<bool> *p_mask)
{
  const int nbRows = static_cast<int>(I.getRows()), nbCols = static_cast<int>(I.getCols());
  const int offsetIdiff = 1;
  const int resetCounter = nbCols - 1;
  const int nrows(nbRows - 1);

  int nbThread = maxNbThread;
  if (nbThread < 0) {
    nbThread = omp_get_max_threads();
  }

  if (static_cast<int>(nbRows) < (4 * nbThread)) {
    // Not enough rows to split the image between the threads
    gradientFilterYMonothread(I, GI, filter, p_mask);
    return;
  }

  int iam, nt, irows, rstart, istart, istop;

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#endif
#pragma omp parallel default(shared) private(iam, nt, irows, rstart, istart, istop) num_threads(nbThread)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    irows = nrows / nt;
    // size of partition
    rstart = irows * iam;
    istart = rstart * nbCols; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      irows = nrows - rstart;
    }
    istop = istart + irows * nbCols;

    std::vector<OutputType> Idiff(static_cast<size_t>((irows + 2) * nbCols));
    std::vector<OutputType> GItemp(static_cast<size_t>(irows * nbCols));

    if (iam == 0) {
      initGradientFilterDifferenceImageY(I, Idiff);
    }
    else {
      initGradientFilterDifferenceImageY(I, Idiff, istart);
    }

    int counter = resetCounter, iterSign = offsetIdiff;
    int iterStart = (iam != 0 ? istart : istart + nbCols);
    for (int iter = iterStart; iter < istop; ++iter) {
      // Computing the amplitude of the difference
      OutputType futureDiff = 0.;

      if (checkBooleanPatch(p_mask, iter + offsetIdiff, iterSign, nbRows, nbCols, false)) {
        futureDiff = static_cast<OutputType>(I.bitmap[static_cast<size_t>(iter + nbCols +1)].V - I.bitmap[static_cast<size_t>(iter + offsetIdiff)].V);
        Idiff[static_cast<size_t>(iter - istart + nbCols + offsetIdiff)] = futureDiff;
      }
      if (counter) {
        if ((counter != resetCounter)) {
          if (checkBooleanMask(p_mask, iter)) {
            OutputType gradient = 0.;
            for (int i = -1; i <= 1; ++i) {
              // Kind of +/- (I[r + 1][c + i] - I[r][c + 1]) +/- (I[r][c + i] - I[r - 1][c + 1])
              gradient += filter[static_cast<size_t>(i + 1)] * (Idiff[static_cast<size_t>(iter - istart + nbCols + i)] + Idiff[static_cast<size_t>(iter - istart + i)]);
            }
            GItemp[static_cast<size_t>(iter - istart)] = gradient;
          }
        }
        --counter;
      }
      else {
        counter = resetCounter;
      }
      if (iterSign < resetCounter) {
        ++iterSign;
      }
      else {
        iterSign = 0;
      }
    }
#pragma omp critical
    {
      std::memcpy(GI.bitmap + istart, GItemp.data(), GItemp.size() * sizeof(OutputType));
    }
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
  }
}
#endif
#endif

inline unsigned char filterGaussXPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
{
  const int val2 = 2;
  return static_cast<unsigned char>(((1. * I[i][j - val2]) + (4. * I[i][j - 1]) + (6. * I[i][j]) + (4. * I[i][j + 1]) + (1. * I[i][j + val2])) / 16.);
}
inline unsigned char filterGaussYPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
{
  const int val2 = 2;
  return static_cast<unsigned char>(((1. * I[i - val2][j]) + (4. * I[i - 1][j]) + (6. * I[i][j]) + (4. * I[i + 1][j]) + (1. * I[i + val2][j])) / 16.);
}

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
template<typename ImageType, typename FilterType>
inline FilterType filterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template <typename ImageType, typename FilterType>
inline FilterType filterXLeftBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                           const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][i - c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template <typename ImageType, typename FilterType>
inline FilterType filterXRightBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  FilterType result = static_cast<FilterType>(0.);
  const unsigned int twice = 2;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r][((twice * width) - c) - i - 1] + I[r][c - i]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}
#else
template<class Color, typename FilterType>
inline typename std::enable_if<!std::is_same<Color, vpRGBa>::value, void>::type
filterChannel(const Color &in, vpColVector &out, const FilterType &coeff)
{
  out[0] = coeff * vpColorGetter<0>::get(in);
  out[1] = coeff * vpColorGetter<1>::get(in);
  out[2] = coeff * vpColorGetter<2>::get(in);
}

template<class Color, typename FilterType>
inline typename std::enable_if<std::is_same<Color, vpRGBa>::value, void>::type
filterChannel(const Color &in, vpColVector &out, const FilterType &coeff)
{
  out[0] = coeff * vpColorGetter<0>::get(in);
  out[1] = coeff * vpColorGetter<1>::get(in);
  out[2] = coeff * vpColorGetter<2>::get(in);
  out[3] = vpRGBa::alpha_default;
}

template<class Color, typename FilterType>
inline void
filterChannel(const Color &in1, const Color &in2, vpColVector &out, const FilterType &coeff)
{
  out[0] += coeff * (vpColorGetter<0>::get(in1) + vpColorGetter<0>::get(in2));
  out[1] += coeff * (vpColorGetter<1>::get(in1) + vpColorGetter<1>::get(in2));
  out[2] += coeff * (vpColorGetter<2>::get(in1) + vpColorGetter<2>::get(in2));
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = filter[0] * static_cast<FilterType>(I[r][c]);

  for (unsigned int i = 1; i <= stop; ++i) {
    res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
  }
  result = static_cast<FilterType>(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterX(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    filterChannel(I[r][c + i], I[r][c - i], res, filter[i]);
  }
  result = OutputType(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = filter[0] * static_cast<FilterType>(I[r][c]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][i - c]);
    }
  }
  result = static_cast<OutputType>(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterXLeftBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      filterChannel(I[r][c + i], I[r][c - i], res, filter[i]);
    }
    else {
      filterChannel(I[r][c + i], I[r][i - c], res, filter[i]);
    }
  }
  result = OutputType(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  const unsigned int twice = 2;
  FilterType res = filter[0] * static_cast<FilterType>(I[r][c]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      res += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[r][((twice * width) - c) - i - 1] + I[r][c - i]);
    }
  }
  result = static_cast<FilterType>(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterXRightBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  const unsigned int twice = 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      filterChannel(I[r][c + i], I[r][c - i], res, filter[i]);
    }
    else {
      filterChannel(I[r][((twice * width) - c) - i - 1], I[r][c - i], res, filter[i]);
    }
  }
  result = OutputType(res);
}
#endif

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98

template<typename ImageType, typename FilterType>
inline FilterType filterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
inline FilterType filterYTopBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                          const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[i - r][c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
inline FilterType filterYBottomBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);
  const unsigned int twiceHeight = 2 * height;
  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[(twiceHeight - r) - i - 1][c] + I[r - i][c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}
#else
template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = (filter[0] * static_cast<FilterType>(I[r][c]));

  for (unsigned int i = 1; i <= stop; ++i) {
    res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
  }
  result = static_cast<OutputType>(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type
filterY(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    filterChannel(I[r + i][c], I[r - i][c], res, filter[i]);
  }
  result = OutputType(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type
filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType res = (filter[0] * static_cast<FilterType>(I[r][c]));

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[i - r][c]);
    }
  }
  result = static_cast<OutputType>(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYTopBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      filterChannel(I[r + i][c], I[r - i][c], res, filter[i]);
    }
    else {
      filterChannel(I[r + i][c], I[i - r][c], res, filter[i]);
    }
  }
  result = OutputType(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<std::is_arithmetic<ImageType>::value, void>::type filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  const unsigned int twiceHeight = 2 * height;
  FilterType res = (filter[0] * static_cast<FilterType>(I[r][c]));
  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      res += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      res += filter[i] * static_cast<FilterType>(I[(twiceHeight - r) - i - 1][c] + I[r - i][c]);
    }
  }
  result = static_cast<OutputType>(res);
}

template<typename ImageType, typename OutputType, typename FilterType>
inline typename std::enable_if<!std::is_arithmetic<ImageType>::value, void>::type  filterYBottomBorder(const vpImage<ImageType> &I, OutputType &result, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int height = I.getHeight();
  const unsigned int twiceHeight = 2 * height;
#ifdef VISP_HAVE_OPENMP
  vpColVector res(ImageType::nbChannels);
#else
  static vpColVector res(ImageType::nbChannels);
#endif
  filterChannel(I[r][c], res, filter[0]);

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      filterChannel(I[r + i][c], I[r - i][c], res, filter[i]);
    }
    else {
      filterChannel(I[(twiceHeight - r) - i - 1][c], I[r - i][c], res, filter[i]);
    }
  }
  result = OutputType(res);
}
#endif

template <class ImageType> double derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c)
{
  const int val1 = 1, val2 = 2, val3 = 3;
  return ((2047.0 * static_cast<double>(I[r][c + val1] - I[r][c - val1])) + (913.0 * static_cast<double>(I[r][c + val2] - I[r][c - val2])) +
          (112.0 * static_cast<double>(I[r][c + val3] - I[r][c - val3]))) / 8418.0;
}

template <class ImageType> double derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c)
{
  const int val1 = 1, val2 = 2, val3 = 3;
  return ((2047.0 * static_cast<double>(I[r + val1][c] - I[r - val1][c])) + (913.0 * static_cast<double>(I[r + val2][c] - I[r - val2][c])) +
          (112.0 * static_cast<double>(I[r + val3][c] - I[r - val3][c]))) / 8418.0;
}

template <class ImageType, typename FilterType>
FilterType derivativeFilterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  unsigned int i;
  FilterType result;

  result = 0;

  for (i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r][c + i] - I[r][c - i]);
  }
  return result;
}

template <class ImageType, typename FilterType>
FilterType derivativeFilterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  unsigned int i;
  FilterType result;

  result = 0;

  for (i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r + i][c] - I[r - i][c]);
  }
  return result;
}
}

template <typename ImageType, typename FilterType>
inline void computePartialDerivatives(const vpImage<ImageType> &I,
                                             vpImage<FilterType> &dIx, vpImage<FilterType> &dIy,
                                             const bool &computeDx, const bool &computeDy, const bool &normalize,
                                             const unsigned int &gaussianKernelSize, const FilterType &gaussianStdev,
                                             const unsigned int &apertureGradient,
                                             const vpCannyFilteringAndGradientType &filteringType,
                                             const vpCannyBackendType &backend,
                                             const vpImage<bool> *p_mask)
{
  if (backend == CANNY_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
    cv::Mat cv_I, cv_dIx, cv_dIy;
    vpImageConvert::convert(I, cv_I);
    computePartialDerivatives(cv_I, cv_dIx, cv_dIy, computeDx, computeDy, normalize, gaussianKernelSize,
                              static_cast<float>(gaussianStdev), apertureGradient, filteringType);
    if (computeDx) {
      vpImageConvert::convert(cv_dIx, dIx);
    }
    if (computeDy) {
      vpImageConvert::convert(cv_dIy, dIy);
    }
#else
    throw(vpException(vpException::badValue, "You need to compile ViSP with OpenCV to use CANNY_OPENCV_BACKEND"));
#endif
  }
  else {
    if ((filteringType == CANNY_GBLUR_SCHARR_FILTERING) || (filteringType == CANNY_GBLUR_SOBEL_FILTERING)) {
      dIx.resize(I.getHeight(), I.getWidth());
      dIy.resize(I.getHeight(), I.getWidth());

      // Computing the Gaussian blur + gradients of the image
      vpImage<FilterType> Iblur;
      gaussianBlur(I, Iblur, gaussianKernelSize, gaussianStdev, true, p_mask);

      vpArray2D<FilterType> gradientFilterX(apertureGradient, apertureGradient); // Gradient filter along the X-axis
      vpArray2D<FilterType> gradientFilterY(apertureGradient, apertureGradient); // Gradient filter along the Y-axis

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
        // Helper to apply the scale to the raw values of the filters
      auto scaleFilter = [](vpArray2D<FilterType> &filter, const FilterType &scale) {
        const unsigned int nbRows = filter.getRows();
        const unsigned int nbCols = filter.getCols();
        for (unsigned int r = 0; r < nbRows; ++r) {
          for (unsigned int c = 0; c < nbCols; ++c) {
            filter[r][c] = filter[r][c] * scale;
          }
        }
        };
#endif

        // Scales to apply to the filters to get a normalized gradient filter that gives a gradient
        // between 0 and 255 for an vpImage<uchar>
      FilterType scaleX = 1.0;
      FilterType scaleY = 1.0;

      if (filteringType == CANNY_GBLUR_SOBEL_FILTERING) {
        if (computeDx) {
          scaleX = static_cast<FilterType>(getSobelKernelX(gradientFilterX, apertureGradient));
        }
        if (computeDy) {
          scaleY = static_cast<FilterType>(getSobelKernelY(gradientFilterY, apertureGradient));
        }
      }
      else if (filteringType == CANNY_GBLUR_SCHARR_FILTERING) {
        if (computeDx) {
          scaleX = static_cast<FilterType>(getScharrKernelX(gradientFilterX, apertureGradient));
        }
        if (computeDy) {
          scaleY = static_cast<FilterType>(getScharrKernelY(gradientFilterY, apertureGradient));
        }
      }

      // Scale the gradient filters to have a normalized gradient filter
      if (normalize) {
        if (computeDx) {
          scaleFilter(gradientFilterX, scaleX);
        }
        if (computeDy) {
          scaleFilter(gradientFilterY, scaleY);
        }
      }

      // Apply the gradient filters to get the gradients
      if (computeDx) {
        filter(Iblur, dIx, gradientFilterX, true, p_mask);
      }

      if (computeDy) {
        filter(Iblur, dIy, gradientFilterY, true, p_mask);
      }
    }
    else {
      std::string errMsg = "[computePartialDerivatives] Filtering + gradient method \"";
      errMsg += vpCannyFiltAndGradTypeToStr(filteringType);
      errMsg += "\" is not implemented yet\n";
      throw(vpException(vpException::notImplementedError, errMsg));
    }
  }
}

template<typename OutType>
inline float computeCannyThreshold(const vpImage<unsigned char> &I, float &lowerThresh,
                                          const vpImage<OutType> *p_dIx, const vpImage<OutType> *p_dIy,
                                          const unsigned int &gaussianKernelSize,
                                          const OutType &gaussianStdev, const unsigned int &apertureGradient,
                                          const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                          const vpCannyFilteringAndGradientType &filteringType,
                                          const vpImage<bool> *p_mask)
{
  const unsigned int w = I.getWidth();
  const unsigned int h = I.getHeight();
  const int size = static_cast<int>(I.getSize());

  if ((lowerThresholdRatio <= 0.f) || (lowerThresholdRatio >= 1.f)) {
    std::stringstream errMsg;
    errMsg << "Lower ratio (" << lowerThresholdRatio << ") " << (lowerThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  if ((upperThresholdRatio <= 0.f) || (upperThresholdRatio >= 1.f)) {
    std::stringstream errMsg;
    errMsg << "Upper ratio (" << upperThresholdRatio << ") " << (upperThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  if (lowerThresholdRatio  >= upperThresholdRatio) {
    std::stringstream errMsg;
    errMsg << "Lower ratio (" << lowerThresholdRatio << ") should be lower than the upper ratio (" << upperThresholdRatio << ")";
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  vpImage<unsigned char> dI(h, w);
  vpImage<OutType> dIx(h, w), dIy(h, w);
  if ((p_dIx != nullptr) && (p_dIy != nullptr)) {
    dIx = *p_dIx;
    dIy = *p_dIy;
  }
  else {
    computePartialDerivatives(I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev,
                              apertureGradient, filteringType, CANNY_VISP_BACKEND, p_mask);
  }

  // Computing the absolute gradient of the image G = |dIx| + |dIy|
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int iter = 0; iter < size; ++iter) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
    bool computeVal = detail::checkBooleanMask(p_mask, iter);

    if (computeVal) {
      float dx = static_cast<float>(dIx.bitmap[iter]);
      float dy = static_cast<float>(dIy.bitmap[iter]);
      float gradient = std::abs(dx) + std::abs(dy);
      float gradientClamped = std::min<float>(gradient, static_cast<float>(std::numeric_limits<unsigned char>::max()));
      dI.bitmap[iter] = static_cast<unsigned char>(gradientClamped);
    }
  }

  // Compute the histogram
  vpHistogram hist;
  hist.setMask(p_mask);
  const unsigned int nbBins = 256;
  hist.calculate(dI, nbBins);
  float totalNbPixels = static_cast<float>(hist.getTotal());
  float accu = 0;
  float t = upperThresholdRatio * totalNbPixels;
  float tLow = lowerThresholdRatio * totalNbPixels;
  float bon = 0;
  unsigned int i = 0;
  bool notFound = true, notFoundLower = true;
  while ((i < nbBins) && notFound) {
    float tf = static_cast<float>(hist[static_cast<unsigned char>(i)]);
    accu = accu + tf;
    if ((accu > tLow) && notFoundLower) {
      lowerThresh = static_cast<float>(i);
      notFoundLower = false;
    }
    if (accu > t) {
      bon = static_cast<float>(i);
      notFound = false;
    }
    ++i;
  }
  if (notFound) {
    std::stringstream errMsg;
    errMsg << "Could not find a bin for which " << upperThresholdRatio * 100.f << " percents of the pixels had a gradient lower than the upper threshold.";
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  float upperThresh = std::max<float>(bon, 1.f);
  lowerThresh = std::max<float>(lowerThresh, std::numeric_limits<float>::epsilon());
  return upperThresh;
}

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template<typename ArithmeticType, bool useFullScale, typename OutType>
inline float computeCannyThreshold(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, float &lowerThresh,
                                          const vpImage<OutType> *p_dIx, const vpImage<OutType> *p_dIy,
                                          const unsigned int &gaussianKernelSize,
                                          const OutType &gaussianStdev,
                                          const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                          const vpCannyFilteringAndGradientType &filteringType,
                                          const vpImage<bool> *p_mask)
{
  const unsigned int w = I.getWidth();
  const unsigned int h = I.getHeight();
  const int size = static_cast<int>(I.getSize());

  if ((lowerThresholdRatio <= 0.f) || (lowerThresholdRatio >= 1.f)) {
    std::stringstream errMsg;
    errMsg << "Lower ratio (" << lowerThresholdRatio << ") " << (lowerThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  if ((upperThresholdRatio <= 0.f) || (upperThresholdRatio >= 1.f)) {
    std::stringstream errMsg;
    errMsg << "Upper ratio (" << upperThresholdRatio << ") " << (upperThresholdRatio < 0.f ? "should be greater than 0 !" : "should be lower than 1 !");
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  if (lowerThresholdRatio  >= upperThresholdRatio) {
    std::stringstream errMsg;
    errMsg << "Lower ratio (" << lowerThresholdRatio << ") should be lower than the upper ratio (" << upperThresholdRatio << ")";
    throw(vpException(vpException::fatalError, errMsg.str()));
  }

  int nbThread = 1;
#ifdef VISP_HAVE_OPENMP
  nbThread = omp_get_max_threads();
#endif

  vpImage<OutType> dI(h, w);
  vpImage<OutType> dIx(h, w), dIy(h, w);
  if ((p_dIx != nullptr) && (p_dIy != nullptr)) {
    dIx = *p_dIx;
    dIy = *p_dIy;
  }
  else {
    vpImage<vpHSV<ArithmeticType, useFullScale>> Iblur;
    gaussianBlur(I, Iblur, gaussianKernelSize, gaussianStdev, true, p_mask);
    gradientFilter(Iblur, dIx, dIy, nbThread, p_mask, filteringType);
  }

  // Computing the absolute gradient of the image G = |dIx| + |dIy|
  float dIMax = -1.; // dI is the absolute gradient => positive
  float dIMin = std::numeric_limits<OutType>::max();
  int iter, istart = 0, istop = size;
#ifdef VISP_HAVE_OPENMP
  int iam, nt, ipoints, npoints(static_cast<int>(size));
#pragma omp parallel default(shared) private(iter, iam, nt, ipoints, istart, istop)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    ipoints = npoints / nt;
    // size of partition
    istart = iam * ipoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      ipoints = npoints - istart;
    }
    istop = istart + ipoints;
#endif
    float localdImin = std::numeric_limits<OutType>::max(), localdImax = -1.;
    for (iter = istart; iter < istop; ++iter) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, iter);

      if (computeVal) {
        float dx = static_cast<float>(dIx.bitmap[iter]);
        float dy = static_cast<float>(dIy.bitmap[iter]);
        float gradient = std::abs(dx) + std::abs(dy);
        localdImax = std::max(localdImax, gradient);
        localdImin = std::min(localdImin, gradient);
        dI.bitmap[iter] = gradient;
      }
    }
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
    {
      dIMin = std::min(dIMin, localdImin);
      dIMax = std::max(dIMax, localdImax);
    }
#else
    dIMin = localdImin;
    dIMax = localdImax;
#endif
#ifdef VISP_HAVE_OPENMP
  }
#endif

    // Compute the histogram
#ifdef VISP_HAVE_THREADS
  nbThread = std::max(static_cast<int>(std::thread::hardware_concurrency()), 1); // The method can return 0 when it is not able to detect the supported number of threads.
#endif
  const unsigned int nbBins = 1024;
  vpHistogram hist(nbBins);
  hist.setMask(p_mask);
  OutType step = 0.;
  hist.calculate<OutType>(dI, dIMin, dIMax, step, nbBins, static_cast<unsigned int>(nbThread));
  float totalNbPixels = static_cast<float>(hist.getTotal());
  float accu = 0;
  float t = upperThresholdRatio * totalNbPixels;
  float tLow = lowerThresholdRatio * totalNbPixels;
  float bon = 0;
  unsigned int i = 0;
  bool notFound = true, notFoundLower = true;
  while ((i < nbBins) && notFound) {
    float tf = static_cast<float>(hist[i]);
    accu = accu + tf;
    if ((accu > tLow) && notFoundLower) {
      lowerThresh = static_cast<float>(i) *  static_cast<float>(step) + dIMin;
      notFoundLower = false;
    }
    if (accu > t) {
      bon = static_cast<float>(i);
      notFound = false;
    }
    ++i;
  }
  if (notFound) {
    std::stringstream errMsg;
    errMsg << "Could not find a bin for which " << upperThresholdRatio * 100.f << " percents of the pixels had a gradient lower than the upper threshold.";
    throw(vpException(vpException::fatalError, errMsg.str()));
  }
  float upperThresh = bon *  static_cast<float>(step) + dIMin;
  lowerThresh = std::max<float>(lowerThresh, std::numeric_limits<float>::epsilon());
  return upperThresh;
}
#endif

template <typename ImageType, typename FilterType>
void filter(const vpImage<ImageType> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve,
                   const vpImage<bool> *p_mask)
{
  const unsigned int size_y = M.getRows(), size_x = M.getCols();
  const unsigned int half_size_y = size_y / 2, half_size_x = size_x / 2;

  const unsigned int inputHeight = I.getHeight(), inputWidth = I.getWidth();
  If.resize(inputHeight, inputWidth, 0.0);

  if (convolve) {
    const unsigned int stopHeight = inputHeight - half_size_y;
    const unsigned int stopWidth = inputWidth - half_size_x;
    for (unsigned int i = half_size_y; i < stopHeight; ++i) {
      for (unsigned int j = half_size_x; j < stopWidth; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          FilterType conv = 0;

          for (unsigned int a = 0; a < size_y; ++a) {
            for (unsigned int b = 0; b < size_x; ++b) {
              FilterType val = static_cast<FilterType>(I[(i + half_size_y) - a][(j + half_size_x) - b]); // Convolution
              conv += M[a][b] * val;
            }
          }
          If[i][j] = conv;
        }
      }
    }
  }
  else {
    const unsigned int stopHeight = inputHeight - half_size_y;
    const unsigned int stopWidth = inputWidth - half_size_x;
    for (unsigned int i = half_size_y; i < stopHeight; ++i) {
      for (unsigned int j = half_size_x; j < stopWidth; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          FilterType corr = 0;

          for (unsigned int a = 0; a < size_y; ++a) {
            for (unsigned int b = 0; b < size_x; ++b) {
              FilterType val = static_cast<FilterType>(I[(i - half_size_y) + a][(j - half_size_x) + b]); // Correlation
              corr += M[a][b] * val;
            }
          }
          If[i][j] = corr;
        }
      }
    }
  }
}

template <typename ImageType, typename FilterType>
void filter(const vpImage<ImageType> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M,
                   bool convolve, const vpImage<bool> *p_mask)
{
  const unsigned int size = M.getRows();
  const unsigned int half_size = size / 2;
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stopV = height - half_size;
  const unsigned int stopU = width - half_size;

  Iu.resize(height, width, 0.0);
  Iv.resize(height, width, 0.0);

  if (convolve) {
    for (unsigned int v = half_size; v < stopV; ++v) {
      for (unsigned int u = half_size; u < stopU; ++u) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, v, u);
        if (computeVal) {
          FilterType conv_u = 0;
          FilterType conv_v = 0;

          for (unsigned int a = 0; a < size; ++a) {
            for (unsigned int b = 0; b < size; ++b) {
              FilterType val = static_cast<FilterType>(I[(v + half_size) - a][(u + half_size) - b]); // Convolution
              conv_u += M[a][b] * val;
              conv_v += M[b][a] * val;
            }
          }
          Iu[v][u] = conv_u;
          Iv[v][u] = conv_v;
        }
      }
    }
  }
  else {
    for (unsigned int v = half_size; v < stopV; ++v) {
      for (unsigned int u = half_size; u < stopU; ++u) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, v, u);

        if (computeVal) {
          FilterType conv_u = 0;
          FilterType conv_v = 0;

          for (unsigned int a = 0; a < size; ++a) {
            for (unsigned int b = 0; b < size; ++b) {
              FilterType val = static_cast<FilterType>(I[(v - half_size) + a][(u - half_size) + b]); // Correlation
              conv_u += M[a][b] * val;
              conv_v += M[b][a] * val;
            }
          }
          Iu[v][u] = conv_u;
          Iv[v][u] = conv_v;
        }
      }
    }
  }
}

void sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH, const vpColVector &kernelV);

template <typename ImageType, typename FilterType>
void filter(const vpImage<ImageType> &I, vpImage<FilterType> &GI, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask)
{
  vpImage<FilterType> GIx;
  filterX<ImageType, FilterType, FilterType>(I, GIx, filter, size, p_mask);
  filterY<FilterType, FilterType, FilterType>(GIx, GI, filter, size, p_mask);
  GIx.destroy();
}

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
template <typename ImageType, typename FilterType>
void filterX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight();
  const unsigned int width = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = width - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::filterXLeftBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::filterX<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop2J; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::filterXRightBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
}

#else
template<typename ImageType, typename OutputType, typename FilterType>
void filterX(const vpImage<ImageType> &I, vpImage<OutputType> &dIx, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight();
  const unsigned int width = I.getWidth();
  const int stop1J = static_cast<int>((size - 1) / 2);
  const int stop2J = static_cast<int>(width - ((size - 1) / 2));
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  int istart = 0;
  int istop = static_cast<int>(height);
#ifdef VISP_HAVE_OPENMP
  int iam, nt, ipoints, npoints(static_cast<int>(height));
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart, istop)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    ipoints = npoints / nt;
    // size of partition
    istart = iam * ipoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      ipoints = npoints - istart;
    }
    istop = istart + ipoints;
#endif
    for (int i = istart; i < istop; ++i) {
      for (int j = 0; j < stop1J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (computeVal) {
          detail::filterXLeftBorder(I, dIx[static_cast<unsigned int>(i)][static_cast<unsigned int>(j)],
                                           static_cast<unsigned int>(i), static_cast<unsigned int>(j), filter, size);
        }
      }

      for (int j = stop1J; j < stop2J; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (computeVal) {
          detail::filterX(I, dIx[i][j], static_cast<unsigned int>(i), static_cast<unsigned int>(j), filter, size);
        }
      }
      for (int j = stop2J; j < static_cast<int>(width); ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (computeVal) {
          detail::filterXRightBorder(I, dIx[static_cast<unsigned int>(i)][static_cast<unsigned int>(j)],
                                            static_cast<unsigned int>(i), static_cast<unsigned int>(j), filter, size);
        }
      }
    }
#ifdef VISP_HAVE_OPENMP
  }
#endif
}

#endif


// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if ((__cplusplus <= 199711L) || (defined(_MSVC_LANG) && (_MSVC_LANG == 199711L))) // Check if cxx98
template<typename ImageType, typename FilterType>
void filterY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::filterYTopBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::filterY<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop2I; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::filterYBottomBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
}

#else

template<typename ImageType, typename OutputType, typename FilterType>
void filterY(const vpImage<ImageType> &I, vpImage<OutputType> &dIy, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  unsigned int jstart = 0;
  unsigned int jstop = width;
#ifdef VISP_HAVE_OPENMP
  unsigned int iam, nt, jpoints, npoints(width);
#pragma omp parallel default(shared) private(iam, nt, jpoints, jstart, jstop)
  {
    iam = static_cast<unsigned int>(omp_get_thread_num());
    nt = static_cast<unsigned int>(omp_get_num_threads());
    jpoints = npoints / nt;
    // size of partition
    jstart = iam * jpoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      jpoints = npoints - jstart;
    }
    jstop = jstart + jpoints;
#endif
    for (unsigned int i = 0; i < stop1I; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
        // We have to compute the value for each pixel if we don't have a mask or for
        // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          detail::filterYTopBorder(I, dIy[i][j], i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop1I; i < stop2I; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          detail::filterY(I, dIy[i][j], i, j, filter, size);
        }
      }
    }
    for (unsigned int i = stop2I; i < height; ++i) {
      for (unsigned int j = jstart; j < jstop; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
        bool computeVal = detail::checkBooleanMask(p_mask, i, j);
        if (computeVal) {
          detail::filterYBottomBorder(I, dIy[i][j], i, j, filter, size);
        }
      }
#ifdef VISP_HAVE_OPENMP
    }
#endif
  }
}
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename ImageType, typename OutputType, typename FilterType>
inline void
gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size, FilterType sigma, bool normalize,
                        const vpImage<bool> *p_mask)
{
  if (size == 0 || size-1 > I.getWidth() || size-1 > I.getHeight()) {
    std::ostringstream oss;
    oss << "Image size (" << I.getWidth() << "x" << I.getHeight() << ") is too small for the Gaussian kernel ("
      << "size=" << size << "), min size is " << (size-1);
    throw vpException(vpException::dimensionError, oss.str());
  }

  FilterType *fg = new FilterType[(size + 1) / 2];
  getGaussianKernel<FilterType>(fg, size, sigma, normalize);
  vpImage<OutputType> GIx;
  filterX<ImageType, OutputType>(I, GIx, fg, size, p_mask);
  filterY<OutputType, OutputType>(GIx, GI, fg, size, p_mask);
  GIx.destroy();
  delete[] fg;
}
#else
template <typename ImageType, typename OutputType>
inline void
gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size, float sigma, bool normalize,
                        const vpImage<bool> *p_mask)
{
  gaussianBlur<ImageType, OutputType, float>(I, GI, size, sigma, normalize, p_mask);
}

template <typename ImageType, typename OutputType, typename FilterType >
inline void
gaussianBlur(const vpImage<ImageType> &I, vpImage<OutputType> &GI, unsigned int size, FilterType sigma, bool normalize,
                        const vpImage<bool> *p_mask)
{
  if (size == 0 || size-1 > I.getWidth() || size-1 > I.getHeight()) {
    std::ostringstream oss;
    oss << "Image size (" << I.getWidth() << "x" << I.getHeight() << ") is too small for the Gaussian kernel ("
      << "size=" << size << "), min size is " << (size-1);
    throw vpException(vpException::dimensionError, oss.str());
  }

  FilterType *fg = new FilterType[(size + 1) / 2];
  getGaussianKernel<FilterType>(fg, size, sigma, normalize);
  vpImage<OutputType> GIx;
  filterX<ImageType, OutputType>(I, GIx, fg, size, p_mask);
  filterY<OutputType, OutputType>(GIx, GI, fg, size, p_mask);
  GIx.destroy();
  delete[] fg;
}
#endif

template<typename FilterType>
void getGaussianKernel(FilterType *filter, unsigned int size, FilterType sigma, bool normalize)
{
  const unsigned int mod2 = 2;
  if ((size % mod2) != 1) {
    throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));
  }

  if (sigma <= 0) {
    sigma = static_cast<FilterType>((size - 1) / 6.0);
  }

  // Utilisation de unsigned int
  unsigned int middle = (size - 1) / 2;
  FilterType sigma2 = static_cast<FilterType>(vpMath::sqr(static_cast<double>(sigma)));
  FilterType coef1 = static_cast<FilterType>(1. / (static_cast<double>(sigma) * sqrt(2. * M_PI)));
  FilterType v_2_sigma2 = static_cast<FilterType>(2. * static_cast<double>(sigma2));

  for (unsigned int i = 0; i <= middle; ++i) {
    filter[i] = coef1 * static_cast<FilterType>(exp(static_cast<double>(-(double)(i * i) / v_2_sigma2)));
  }

  if (normalize) {
    // renormalization
    FilterType sum = filter[0];
    for (unsigned int i = 1; i <= middle; ++i) {
      sum += static_cast<FilterType>(2) * filter[i];
    }
    for (unsigned int i = 0; i <= middle; ++i) {
      filter[i] = filter[i] / sum;
    }
  }
}

template <typename FilterType>
void getGaussianDerivativeKernel(FilterType *filter, unsigned int size, FilterType sigma, bool normalize)
{
  const unsigned int mod2 = 2;
  if ((size % mod2) != 1) {
    throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));
  }

  if (sigma <= 0) {
    sigma = static_cast<FilterType>((size - 1) / 6.0);
  }

  const unsigned int half = 2;
  unsigned int middle = (size - 1) / half;
  FilterType sigma2 = static_cast<FilterType>(vpMath::sqr(static_cast<double>(sigma)));
  FilterType coef_1 = static_cast<FilterType>(1. / (static_cast<double>(sigma) * sqrt(2. * M_PI)));
  FilterType coef_1_over_2 = coef_1 / static_cast<FilterType>(2.);
  FilterType v_2_coef_1 = static_cast<FilterType>(2.) * coef_1;
  FilterType v_2_sigma2 = static_cast<FilterType>(2.) * sigma2;

  filter[0] = 0.;
  for (unsigned int i = 1; i <= middle; ++i) {
    FilterType i_plus_1 = static_cast<FilterType>(i + 1);
    FilterType i_minus_1 = static_cast<FilterType>(i - 1);
    filter[i] = -coef_1_over_2 * (static_cast<FilterType>(exp(-(double)(i_plus_1 * i_plus_1) / v_2_sigma2)) -
                                  static_cast<FilterType>(exp(-(double)(i_minus_1 * i_minus_1) / v_2_sigma2)));
  }

  if (normalize) {
    FilterType sum = static_cast<FilterType>(0);
    for (unsigned int i = 1; i <= middle; ++i) {
      FilterType i_ = static_cast<FilterType>(i);
      sum += v_2_coef_1 * static_cast<FilterType>(exp(-static_cast<double>(i_ * i_ / v_2_sigma2)));
    }
    sum += coef_1;

    for (unsigned int i = 1; i <= middle; ++i) {
      filter[i] = filter[i] / sum;
    }
  }
}

// Gradient along X
template<typename FilterType>
void getGradX(const vpImage<unsigned char> &I, vpImage<FilterType> &dIx, const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stopJ = width - 3;
  const unsigned int val_3 = 3;
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      // If a mask is used, the image is already initialized with 0s
      bool computeVal = (p_mask == nullptr);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(0);
      }
    }
    for (unsigned int j = 3; j < stopJ; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(detail::derivativeFilterX(I, i, j));
      }
    }
    for (unsigned int j = stopJ; j < width; ++j) {
      // If a mask is used, the image is already initialized with 0s
      bool computeVal = (p_mask == nullptr);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(0);
      }
    }
  }
}

template <typename ImageType, typename FilterType>
void getGradX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = width - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // If a mask is used, the image is already initialized with 0s
      bool computeVal = (p_mask == nullptr);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(0);
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = detail::derivativeFilterX<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop2J; j < width; ++j) {
      // If a mask is used, the image is already initialized with 0s
      bool computeVal = (p_mask == nullptr);
      if (computeVal) {
        dIx[i][j] = static_cast<FilterType>(0);
      }
    }
  }
}

template <typename ImageType, typename FilterType>
void getGradXGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *gaussianKernel,
                            const FilterType *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask)
{
  vpImage<FilterType> GIy;
  filterY<ImageType, FilterType>(I, GIy, gaussianKernel, size, p_mask);
  getGradX<FilterType, FilterType>(GIy, dIx, gaussianDerivativeKernel, size, p_mask);
}

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterX(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, const int &nbThread, const vpImage<bool> *p_mask, const vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  std::vector<FilterType> filter(3);
  FilterType scale;
  switch (type) {
  case CANNY_GBLUR_SOBEL_FILTERING:
    filter = { 1., 2., 1. };
    scale = 8.;
    break;
  case CANNY_GBLUR_SCHARR_FILTERING:
    filter = { 3., 10., 3. };
    scale = 32.;
    break;
  default:
    throw(vpException(vpException::badValue, "Wrong type of filtering"));
  }

  for (unsigned char i = 0; i < 3; ++i) {
    filter[i] = filter[i] / scale;
  }

#ifdef VISP_HAVE_OPENMP
  if (nbThread == 1) {
    detail::gradientFilterXMonothread(I, GIx, filter, p_mask);
  }
  else {
    detail::gradientFilterXMultithread(I, GIx, filter, nbThread, p_mask);
  }
#else
  (void)nbThread; // Unused paramter when OpenMP is unavailable
  detail::gradientFilterXMonothread(I, GIx, filter, p_mask);
#endif
}
#endif

  // Gradient along Y
template <typename FilterType>
void getGradY(const vpImage<unsigned char> &I, vpImage<FilterType> &dIy, const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stopI = height - 3;
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(0);
      }
    }
  }
  for (unsigned int i = 3; i < stopI; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(detail::derivativeFilterY(I, i, j));
      }
    }
  }
  for (unsigned int i = stopI; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(0);
      }
    }
  }
}

template <typename ImageType, typename FilterType>
void getGradY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size, const vpImage<bool> *p_mask)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  detail::resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(0);
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = detail::derivativeFilterY<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop2I; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = static_cast<FilterType>(0);
      }
    }
  }
}

template <typename ImageType, typename FilterType>
void getGradYGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *gaussianKernel,
                            const FilterType *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask)
{
  vpImage<FilterType> GIx;
  filterX<ImageType, FilterType>(I, GIx, gaussianKernel, size, p_mask);
  getGradY<FilterType, FilterType>(GIx, dIy, gaussianDerivativeKernel, size, p_mask);
}

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilterY(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIy, const int &nbThread, const vpImage<bool> *p_mask, const vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIy.resize(nbRows, nbCols);
  std::vector<FilterType> filter(3);
  FilterType scale;
  switch (type) {
  case CANNY_COUNT_FILTERING:
    // Prewitt case
    filter = { 1., 1., 1. };
    scale = 6.;
    break;
  case CANNY_GBLUR_SOBEL_FILTERING:
    filter = { 1., 2., 1. };
    scale = 8.;
    break;
  case CANNY_GBLUR_SCHARR_FILTERING:
    filter = { 3., 10., 3. };
    scale = 32.;
    break;
  default:
    throw(vpException(vpException::badValue, "Wrong type of filtering"));
  }
  for (unsigned char i = 0; i < 3; ++i) {
    filter[i] = filter[i] / scale;
  }

#ifdef VISP_HAVE_OPENMP
  if (nbThread == 1) {
    detail::gradientFilterYMonothread(I, GIy, filter, p_mask);
  }
  else {
    detail::gradientFilterYMultithread(I, GIy, filter, nbThread, p_mask);
  }
#else
  (void)nbThread; // Unused paramter when OpenMP is unavailable
  detail::gradientFilterYMonothread(I, GIy, filter, p_mask);
#endif
}
#endif

template <typename FilterType>
inline FilterType getScharrKernelX(vpArray2D<FilterType> &filter, unsigned int size)
{
  if (size != 3) {
    std::stringstream errMsg;
    errMsg << "Cannot get Scharr kernel of size " << size << " != 3";
    throw vpException(vpException::dimensionError, errMsg.str());
  }

  filter.resize(size, size);

  vpArray2D<FilterType> ScharrY(size, size);
  FilterType norm = getScharrKernelY<FilterType>(ScharrY, size);
  memcpy(filter.data, ScharrY.t().data, ScharrY.getRows() * ScharrY.getCols() * sizeof(FilterType));
  return norm;
}

template <typename FilterType>
inline FilterType getScharrKernelY(vpArray2D<FilterType> &filter, unsigned int size)
{
  // Scharr kernel pre-computed for the usual size
  static const FilterType ScharrY3x3[9] = { -3.0, -10.0, -3.0, 0.0, 0.0, 0.0, 3.0, 10.0, 3.0 };

  if (size != 3) {
    std::stringstream errMsg;
    errMsg << "Cannot get Scharr kernel of size " << size << " != 3";
    throw vpException(vpException::dimensionError, errMsg.str());
  }

  filter.resize(size, size);

  const unsigned int kernel3 = 3;
  if (size == kernel3) {
    memcpy(filter.data, ScharrY3x3, size * size * sizeof(FilterType));
    return static_cast<FilterType>(1.0 / 32.0);
  }

  return static_cast<FilterType>(0.);
}

template <typename FilterType>
inline FilterType getSobelKernelX(vpArray2D<FilterType> &filter, unsigned int size)
{
  const unsigned int maxSize = 41;
  if (size == 0) {
    throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
  }
  if (size > maxSize) {
    throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > %d!", maxSize);
  }

  filter.resize(size, size);

  vpArray2D<FilterType> SobelY(size, size);
  FilterType norm = getSobelKernelY<FilterType>(SobelY, size);
  memcpy(filter.data, SobelY.t().data, SobelY.getRows() * SobelY.getCols() * sizeof(FilterType));
  return norm;
}

template <typename FilterType>
inline FilterType getSobelKernelY(vpArray2D<FilterType> &filter, unsigned int size)
{
  // Sobel kernel pre-computed for the usual size
  static const FilterType SobelY3x3[9] = { -1.0, -2.0, -1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0 };
  static const FilterType SobelY5x5[25] = { -1.0, -4.0, -6.0, -4.0, -1.0, -2.0, -8.0, -12.0, -8.0, -2.0, 0.0, 0.0, 0.0,
                                      0.0,  0.0,  2.0,  8.0,  12.0, 8.0,  2.0,  1.0,   4.0,  6.0,  4.0, 1.0 };
  static const FilterType SobelY7x7[49] = { -1,   -6,  -15, -20, -15, -6, -1, -4, -24, -60, -80, -60, -24, -4, -5,  -30, -75,
                                      -100, -75, -30, -5,  0,   0,  0,  0,  0,   0,   0,   5,   30,  75, 100, 75,  30,
                                      5,    4,   24,  60,  80,  60, 24, 4,  1,   6,   15,  20,  15,  6,  1 };
  const vpArray2D<FilterType> smoothingKernel(3, 3);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  smoothingKernel[index_0][index_0] = 1.0;
  smoothingKernel[index_0][index_1] = 2.0;
  smoothingKernel[index_0][index_2] = 1.0;
  smoothingKernel[index_1][index_0] = 2.0;
  smoothingKernel[index_1][index_1] = 4.0;
  smoothingKernel[index_1][index_2] = 2.0;
  smoothingKernel[index_2][index_0] = 1.0;
  smoothingKernel[index_2][index_1] = 2.0;
  smoothingKernel[index_2][index_2] = 1.0;

  const unsigned int maxSize = 41;
  if (size == 0) {
    throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
  }
  if (size > maxSize) {
    throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > %d!", maxSize);
  }

  filter.resize(size, size);

  FilterType scale = static_cast<FilterType>(1. / 8.); // Scale to normalize Sobel3x3
  const unsigned int kernel3 = 3, kernel5 = 5, kernel7 = 7;
  if (size == kernel3) {
    memcpy(filter.data, SobelY3x3, size * size * sizeof(FilterType));
    return scale;
  }
  scale *= static_cast<FilterType>(1. / 16.); // Sobel5x5 is the convolution of smoothingKernel, which needs 1/16 scale factor, with Sobel3x3
  if (size == kernel5) {
    memcpy(filter.data, SobelY5x5, size * size * sizeof(FilterType));
    return scale;
  }
  scale *= static_cast<FilterType>(1. / 16.); // Sobel7x7 is the convolution of smoothingKernel, which needs 1/16 scale factor, with Sobel5x5
  if (size == kernel7) {
    memcpy(filter.data, SobelY7x7, size * size * sizeof(FilterType));
    return scale;
  }

  vpArray2D<FilterType> sobelY(7, 7);
  memcpy(sobelY.data, SobelY7x7, sobelY.getRows() * sobelY.getCols() * sizeof(FilterType));
  for (unsigned int i = 4; i <= size; ++i) {
    sobelY = vpArray2D<FilterType>::conv2(sobelY, smoothingKernel, "full");
    // Sobel(N+1)x(N+1) is the convolution of smoothingKernel, which needs 1/16 scale factor, with SobelNxN
    scale *= static_cast<FilterType>(1. / 16.);
  }

  memcpy(filter.data, sobelY.data, sobelY.getRows() * sobelY.getCols() * sizeof(FilterType));

  return scale;
}

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
template <typename ArithmeticType, typename FilterType, bool useFullScale>
void gradientFilter(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, vpImage<FilterType> &GIx, vpImage<FilterType> &GIy, const int &nbThread, const vpImage<bool> *p_mask, const vpCannyFilteringAndGradientType &type)
{
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  GIx.resize(nbRows, nbCols, 0.);
  GIy.resize(nbRows, nbCols, 0.);
  gradientFilterX(I, GIx, nbThread, p_mask, type);
  gradientFilterY(I, GIy, nbThread, p_mask, type);
}
#endif
}
#endif
#if defined(__clang__)
#  pragma clang diagnostic pop
#endif
END_VISP_NAMESPACE
#endif
