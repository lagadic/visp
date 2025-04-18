/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 */

#include <visp3/core/vpCannyEdgeDetection.h>

#include <visp3/core/vpImageConvert.h>

#ifdef VISP_HAVE_OPENMP
#include <thread>
#include <omp.h>
#endif

#ifdef VISP_USE_MSVC
#pragma comment(linker, "/STACK:65532000") // Increase max recursion depth
#endif

#if (VISP_CXX_STANDARD == VISP_CXX_STANDARD_98) // Check if cxx98
namespace
{
// Helper to apply the scale to the raw values of the filters
template <typename FilterType>
static void scaleFilter(
#ifdef ENABLE_VISP_NAMESPACE
  visp::
#endif
  vpArray2D<FilterType> &filter, const float &scale)
{
  const unsigned int nbRows = filter.getRows();
  const unsigned int nbCols = filter.getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      filter[r][c] = filter[r][c] * scale;
    }
  }
}
}
#endif

BEGIN_VISP_NAMESPACE
// // Initialization methods

vpImage<vpCannyEdgeDetection::EdgeType> vpCannyEdgeDetection::m_edgePointsCandidates;

vpCannyEdgeDetection::vpCannyEdgeDetection()
  : m_filteringAndGradientType(vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
  , m_nbThread(-1)
  , m_gaussianKernelSize(3)
  , m_gaussianStdev(1.f)
  , m_areGradientAvailable(false)
  , m_gradientFilterKernelSize(3)
  , m_lowerThreshold(-1.f)
  , m_lowerThresholdRatio(0.6f)
  , m_upperThreshold(-1.f)
  , m_upperThresholdRatio(0.8f)
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  , m_minStackSize(0)  // Deactivated by default
#endif
  , mp_mask(nullptr)
{
  reinit();
}

vpCannyEdgeDetection::vpCannyEdgeDetection(const int &gaussianKernelSize, const float &gaussianStdev,
                                           const unsigned int &sobelAperture, const float &lowerThreshold,
                                           const float &upperThreshold, const float &lowerThresholdRatio,
                                           const float &upperThresholdRatio,
                                           const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
                                           const bool &storeEdgePoints, const int &nbThread
)
  : m_filteringAndGradientType(filteringType)
  , m_nbThread(nbThread)
  , m_gaussianKernelSize(gaussianKernelSize)
  , m_gaussianStdev(gaussianStdev)
  , m_areGradientAvailable(false)
  , m_gradientFilterKernelSize(sobelAperture)
  , m_lowerThreshold(lowerThreshold)
  , m_lowerThresholdRatio(lowerThresholdRatio)
  , m_upperThreshold(upperThreshold)
  , m_upperThresholdRatio(upperThresholdRatio)
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  , m_minStackSize(0)  // Deactivated by default
#endif
  , m_storeListEdgePoints(storeEdgePoints)
  , mp_mask(nullptr)
{
  reinit();
}

void
vpCannyEdgeDetection::reinit()
{
  setNbThread(m_nbThread);
  initGaussianFilters();
  initGradientFilters();
  mp_mask = nullptr;
  m_areGradientAvailable = false;

  // // Clearing the previous results
  m_edgeCandidateAndGradient.clear();
  m_activeEdgeCandidates.clear();
  m_edgePointsList.clear();
  if (m_edgeMap.getSize() != 0) {
    m_edgeMap.resize(m_edgeMap.getRows(), m_edgeMap.getCols(), 0);
    m_edgePointsCandidates.resize(m_edgeMap.getRows(), m_edgeMap.getCols(), NOT_EDGE);
  }
}

#ifdef VISP_HAVE_NLOHMANN_JSON

using json = nlohmann::json;

vpCannyEdgeDetection::vpCannyEdgeDetection(const std::string &jsonPath)
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  : m_minStackSize(0)  // Deactivated by default
#endif
{
  initFromJSON(jsonPath);
}

void
vpCannyEdgeDetection::initFromJSON(const std::string &jsonPath)
{
  std::ifstream file(jsonPath);
  if (!file.good()) {
    std::stringstream ss;
    ss << "Problem opening file " << jsonPath << ". Make sure it exists and is readable" << std::endl;
    throw vpException(vpException::ioError, ss.str());
  }
  json j;
  try {
    j = json::parse(file);
  }
  catch (json::parse_error &e) {
    std::stringstream msg;
    msg << "Could not parse JSON file : \n";
    msg << e.what() << std::endl;
    msg << "Byte position of error: " << e.byte;
    throw vpException(vpException::ioError, msg.str());
  }
  from_json(j, *this);
  file.close();
  reinit();
}
#endif

void
vpCannyEdgeDetection::initGaussianFilters()
{
  const int val_2 = 2;
  if ((m_gaussianKernelSize % val_2) == 0) {
    throw(vpException(vpException::badValue, "The Gaussian kernel size should be odd"));
  }
  m_fg.resize(1, static_cast<unsigned int>((m_gaussianKernelSize + 1) / val_2));
  vpImageFilter::getGaussianKernel(m_fg.data, static_cast<unsigned int>(m_gaussianKernelSize), m_gaussianStdev, true);
}

void
vpCannyEdgeDetection::initGradientFilters()
{
  const int val_2 = 2;
  if ((m_gradientFilterKernelSize % val_2) != 1) {
    throw vpException(vpException::badValue, "Gradient filters kernel size should be odd.");
  }
  m_gradientFilterX.resize(m_gradientFilterKernelSize, m_gradientFilterKernelSize);
  m_gradientFilterY.resize(m_gradientFilterKernelSize, m_gradientFilterKernelSize);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  auto scaleFilter = [](vpArray2D<float> &filter, const float &scale) {
    unsigned int filter_rows = filter.getRows();
    unsigned int filter_col = filter.getCols();
    for (unsigned int r = 0; r < filter_rows; ++r) {
      for (unsigned int c = 0; c < filter_col; ++c) {
        filter[r][c] = filter[r][c] * scale;
      }
    }
    };
#endif

  float scaleX = 1.f;
  float scaleY = 1.f;

  if (m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING) {
    scaleX = vpImageFilter::getSobelKernelX(m_gradientFilterX.data, (m_gradientFilterKernelSize - 1) / val_2);
    scaleY = vpImageFilter::getSobelKernelY(m_gradientFilterY.data, (m_gradientFilterKernelSize - 1) / val_2);
  }
  else if (m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING) {
    // Compute the Scharr filters
    scaleX = vpImageFilter::getScharrKernelX(m_gradientFilterX.data, (m_gradientFilterKernelSize - 1) / val_2);
    scaleY = vpImageFilter::getScharrKernelY(m_gradientFilterY.data, (m_gradientFilterKernelSize - 1) / val_2);
  }
  else {
    std::string errMsg = "[vpCannyEdgeDetection::initGradientFilters] Error: gradient filtering method \"";
    errMsg += vpImageFilter::vpCannyFiltAndGradTypeToStr(m_filteringAndGradientType);
    errMsg += "\" has not been implemented yet\n";
    throw vpException(vpException::notImplementedError, errMsg);
  }

  scaleFilter(m_gradientFilterX, scaleX);
  scaleFilter(m_gradientFilterY, scaleY);
}

// // Detection methods
#ifdef HAVE_OPENCV_CORE
vpImage<unsigned char>
vpCannyEdgeDetection::detect(const cv::Mat &cv_I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(cv_I, I_gray);
  return detect(I_gray);
}
#endif

vpImage<unsigned char>
vpCannyEdgeDetection::detect(const vpImage<vpRGBa> &I_color)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(I_color, I_gray);
  return detect(I_gray);
}

vpImage<unsigned char>
vpCannyEdgeDetection::detect(const vpImage<unsigned char> &I)
{
  // // Step 1 and 2: filter the image and compute the gradient, if not given by the user
  if (!m_areGradientAvailable) {
    computeFilteringAndGradient(I);
  }
  m_areGradientAvailable = false; // Reset for next call

  // // Step 3: edge thining
  float upperThreshold = m_upperThreshold;
  float lowerThreshold = m_lowerThreshold;
  if (upperThreshold < 0) {
    upperThreshold = vpImageFilter::computeCannyThreshold(I, lowerThreshold, &m_dIx, &m_dIy, m_gaussianKernelSize,
                                                          m_gaussianStdev, m_gradientFilterKernelSize, m_lowerThresholdRatio,
                                                          m_upperThresholdRatio, m_filteringAndGradientType, mp_mask);
  }
  else if (m_lowerThreshold < 0) {
    // Applying Canny recommendation to have the upper threshold 3 times greater than the lower threshold.
    lowerThreshold = m_upperThreshold / 3.f;
  }
  // To ensure that if lowerThreshold = 0, we reject null gradient points
  lowerThreshold = std::max<float>(lowerThreshold, std::numeric_limits<float>::epsilon());

  step3to5(I.getHeight(), I.getWidth(), lowerThreshold, upperThreshold);

  return m_edgeMap;
}

void
vpCannyEdgeDetection::step3to5(const unsigned int &height, const unsigned int &width, const float &lowerThreshold, const float &upperThreshold)
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  rlim_t initialStackSize = 0;
  struct rlimit rl;
  int result;
  if (m_minStackSize > 0) {
    // Check the current stack size
    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0) {
      initialStackSize = rl.rlim_cur;
      if (rl.rlim_cur < m_minStackSize) {
        // Increase stack size due to the recursive algorithm
        rl.rlim_cur = m_minStackSize;
        result = setrlimit(RLIMIT_STACK, &rl);
        if (result != 0) {
          throw(vpException(vpException::fatalError, "setrlimit returned result = %d\n", result));
        }
      }
    }
    else {
      throw(vpException(vpException::fatalError, "getrlimit returned result = %d\n", result));
    }
  }
#endif
  // // Clearing the previous results
  m_edgeMap.resize(height, width, 0);
  m_edgeCandidateAndGradient.clear();
  m_activeEdgeCandidates.clear();
  m_edgePointsCandidates.resize(m_dIx.getRows(), m_dIx.getCols(), NOT_EDGE);
  m_edgePointsList.clear();

  performEdgeThinning(lowerThreshold);

  // // Step 4: hysteresis thresholding
  performHysteresisThresholding(lowerThreshold, upperThreshold);

  // // Step 5: edge tracking
  performEdgeTracking();

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (m_minStackSize > 0) {
    if (rl.rlim_cur > initialStackSize) {
      // Reset stack size to its original value
      rl.rlim_cur = initialStackSize;
      result = setrlimit(RLIMIT_STACK, &rl);
      if (result != 0) {
        throw(vpException(vpException::fatalError, "setrlimit returned result = %d\n", result));

      }
    }
  }
#endif
}

void
vpCannyEdgeDetection::computeFilteringAndGradient(const vpImage<unsigned char> &I)
{
  if ((m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
      || (m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING)) {
    // Computing the Gaussian blur
    vpImage<float> Iblur;
    vpImage<float> GIx;
    vpImageFilter::filterX<unsigned char, float>(I, GIx, m_fg.data, static_cast<unsigned int>(m_gaussianKernelSize), mp_mask);
    vpImageFilter::filterY<float, float>(GIx, Iblur, m_fg.data, static_cast<unsigned int>(m_gaussianKernelSize), mp_mask);

    // Computing the gradients
    vpImageFilter::filter(Iblur, m_dIx, m_gradientFilterX, true, mp_mask);
    vpImageFilter::filter(Iblur, m_dIy, m_gradientFilterY, true, mp_mask);
  }
  else {
    std::string errmsg("Currently, the filtering operation \"");
    errmsg += vpImageFilter::vpCannyFiltAndGradTypeToStr(m_filteringAndGradientType);
    errmsg += "\" is not handled.";
    throw(vpException(vpException::notImplementedError, errmsg));
  }
}

/**
 * \brief Get the interpolation weights and offsets.
 *
 * \param[in] gradientOrientation : The positive value of the angle of the edge, expressed in radians.
 * Its value is between 0 and M_PI_FLOAT radians.
 * \param[out] alpha : The weight of the first point used for the interpolation.
 * \param[out] beta : The weight of the second point used for the interpolation.
 * \param[out] dRowGradAlpha : The offset along the row attached to the alpha weight.
 * \param[out] dRowGradBeta : The offset along the row attached to the beta weight.
 * \param[out] dColGradAlpha : The offset along the column attached to the alpha weight.
 * \param[out] dColGradBeta : The offset along the column attached to the beta weight.
 */
void
vpCannyEdgeDetection::getInterpolWeightsAndOffsets(const float &gradientOrientation,
                                                   float &alpha, float &beta, const int &nbCols,
                                                   int &dRowGradAlpha, int &dRowGradBeta,
                                                   int &dColGradAlpha, int &dColGradBeta
)
{
  float thetaMin = 0.f;
  if (gradientOrientation < M_PI_4_FLOAT) {
    // Angles between 0 and 45 deg rely on the horizontal and diagonal points
    dColGradAlpha = 1;
    dColGradBeta = 1;
    dRowGradAlpha = 0;
    dRowGradBeta = -nbCols;
  }
  else if ((gradientOrientation >= M_PI_4_FLOAT) && (gradientOrientation < M_PI_2_FLOAT)) {
    // Angles between 45 and 90 deg rely on the diagonal and vertical points
    thetaMin = M_PI_4_FLOAT;
    dColGradAlpha = 1;
    dColGradBeta = 0;
    dRowGradAlpha = -nbCols;
    dRowGradBeta = -nbCols;
  }
  else if ((gradientOrientation >= M_PI_2_FLOAT) && (gradientOrientation < (3.f * M_PI_4_FLOAT))) {
    // Angles between 90 and 135 deg rely on the vertical and diagonal points
    thetaMin = M_PI_2_FLOAT;
    dColGradAlpha = 0;
    dColGradBeta = -1;
    dRowGradAlpha = -nbCols;
    dRowGradBeta = -nbCols;
  }
  else if ((gradientOrientation >= (3.f * M_PI_4_FLOAT)) && (gradientOrientation < M_PI_FLOAT)) {
    // Angles between 135 and 180 deg rely on the vertical and diagonal points
    thetaMin = 3.f * M_PI_4_FLOAT;
    dColGradAlpha = -1;
    dColGradBeta = -1;
    dRowGradAlpha = -nbCols;
    dRowGradBeta = 0;
  }
  beta = (gradientOrientation - thetaMin) / M_PI_4_FLOAT;
  alpha = 1.f - beta;
}

/**
 * @brief Get the Manhattan Gradient, i.e. abs(dIx) + abs(dIy) at the index \b row \b col.
 * If one of the index is outside the limits of the image, return 0.
 * @param dIx : Gradient along the horizontal axis.
 * @param dIy : Gradient along the vertical axis.
 * @param row : Index along the vertical axis.
 * @param col : Index along the horizontal axis.
 * @return float grad = abs(dIx) + abs(dIy) if row and col are valid, 0 otherwise.
 */
float
vpCannyEdgeDetection::getManhattanGradient(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &iter)
{
  float grad = 0.;
  float dx = dIx.bitmap[iter];
  float dy = dIy.bitmap[iter];
  grad = std::abs(dx) + std::abs(dy);

  return grad;
}

/**
 * @brief Get the gradient orientation, expressed in radians, between 0 and M_PI_FLOAT radians.
 * If the gradient orientation is negative, we add M_PI radians in
 * order to keep the same orientation but in the positive direction.
 *
 * @param dIx : Gradient along the horizontal axis.
 * @param dIy : Gradient along the vertical axis.
 * @param row : Index along the vertical axis.
 * @param col : Index along the horizontal axis.
 * @return float The positive value of the gradient orientation, expressed in radians.
 */
float
vpCannyEdgeDetection::getGradientOrientation(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &iter)
{
  float gradientOrientation = 0.f;
  float dx = dIx.bitmap[iter];
  float dy = dIy.bitmap[iter];

  if (std::abs(dx) < std::numeric_limits<float>::epsilon()) {
    gradientOrientation = M_PI_2_FLOAT;
  }
  else {
    // -dy because the y-axis of the image is oriented towards the bottom of the screen
    // while we later work with a y-axis oriented towards the top when getting the theta quadrant.
    gradientOrientation = static_cast<float>(std::atan2(-dy, dx));
    if (gradientOrientation < 0.f) {
      gradientOrientation += M_PI_FLOAT; // + M_PI in order to be between 0 and M_PI_FLOAT
    }
  }
  return gradientOrientation;
}

void
vpCannyEdgeDetection::performEdgeThinning(const float &lowerThreshold)
{
  const int nbCols = static_cast<int>(m_dIx.getCols());
  const int size = static_cast<int>(m_dIx.getSize());

  int istart = 0, istop = size;
#ifdef VISP_HAVE_OPENMP
  int iam, nt, ipoints, npoints(size);
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart, istop) num_threads(m_nbThread)
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
    std::vector<std::pair<unsigned int, float>> localMemoryEdgeCandidates;
    bool ignore_current_pixel = false;
    bool grad_lower_threshold = false;
#endif
    for (int iter = istart; iter < istop; ++iter) {
      // reset the checks
      ignore_current_pixel = false;
      grad_lower_threshold = false;

      if (mp_mask != nullptr) {
        if (!mp_mask->bitmap[iter]) {
          // The mask tells us to ignore the current pixel
          ignore_current_pixel = true;
          // continue
        }
      }
      // continue if the mask does not tell us to ignore the current pixel
      if (ignore_current_pixel == false) {

        // Computing the gradient orientation and magnitude
        float grad = getManhattanGradient(m_dIx, m_dIy, iter);

        if (grad < lowerThreshold) {
          // The gradient is lower than minimum threshold => ignoring the point
          grad_lower_threshold = true;
          // continue
        }
        if (grad_lower_threshold == false) {
          //
          // Getting the offset along the horizontal and vertical axes
          // depending on the gradient orientation
          int dRowAlphaPlus = 0, dRowBetaPlus = 0;
          int dColAphaPlus = 0, dColBetaPlus = 0;
          float gradientOrientation = getGradientOrientation(m_dIx, m_dIy, iter);
          float alpha = 0.f, beta = 0.f;
          getInterpolWeightsAndOffsets(gradientOrientation, alpha, beta, nbCols, dRowAlphaPlus, dRowBetaPlus, dColAphaPlus, dColBetaPlus);
          int dRowAlphaMinus = -dRowAlphaPlus, dRowBetaMinus = -dRowBetaPlus;
          int dColAphaMinus = -dColAphaPlus, dColBetaMinus = -dColBetaPlus;
          float gradAlphaPlus = getManhattanGradient(m_dIx, m_dIy, iter + dRowAlphaPlus + dColAphaPlus);
          float gradBetaPlus = getManhattanGradient(m_dIx, m_dIy, iter + dRowBetaPlus + dColBetaPlus);
          float gradAlphaMinus = getManhattanGradient(m_dIx, m_dIy, iter + dRowAlphaMinus + dColAphaMinus);
          float gradBetaMinus = getManhattanGradient(m_dIx, m_dIy, iter + dRowBetaMinus + dColBetaMinus);
          float gradPlus = (alpha * gradAlphaPlus) + (beta * gradBetaPlus);
          float gradMinus = (alpha * gradAlphaMinus) + (beta * gradBetaMinus);

          if ((grad >= gradPlus) && (grad >= gradMinus)) {
            // Keeping the edge point that has the highest gradient
#ifdef VISP_HAVE_OPENMP
            localMemoryEdgeCandidates.push_back(std::pair<unsigned int, float>(iter, grad));
#else
            m_edgeCandidateAndGradient.push_back(std::pair<unsigned int, float>(iter, grad));
#endif
          }
        }
      }
    }
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
    {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      m_edgeCandidateAndGradient.insert(
        m_edgeCandidateAndGradient.end(),
        std::make_move_iterator(localMemoryEdgeCandidates.begin()),
        std::make_move_iterator(localMemoryEdgeCandidates.end())
      );
#else
      m_edgeCandidateAndGradient.insert(
        m_activeEdgeCandidates.end(),
        localMemoryEdgeCandidates.begin(),
        localMemoryEdgeCandidates.end()
      );
#endif
    }
#endif
#ifdef VISP_HAVE_OPENMP
  }
#endif
}

void
vpCannyEdgeDetection::performHysteresisThresholding(const float &lowerThreshold, const float &upperThreshold)
{
  const unsigned int size = m_edgeCandidateAndGradient.size();
  unsigned int istart = 0;
  unsigned int istop = size;

#ifdef VISP_HAVE_OPENMP
  unsigned int iam, nt, ipoints, npoints(size);
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart, istop) num_threads(m_nbThread)
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
    std::vector<unsigned int> localMemoryEdgeCandidates;
#endif
    for (unsigned int id = istart; id < istop; ++id) {
      const std::pair<unsigned int, float> &candidate = m_edgeCandidateAndGradient[id];
      if (candidate.second >= upperThreshold) {
#ifdef VISP_HAVE_OPENMP
        localMemoryEdgeCandidates.push_back(candidate.first);
#else
        m_activeEdgeCandidates.push_back(candidate.first);
#endif
        m_edgePointsCandidates.bitmap[candidate.first] = STRONG_EDGE;
      }
      else if ((candidate.second >= lowerThreshold) && (candidate.second < upperThreshold)) {
#ifdef VISP_HAVE_OPENMP
        localMemoryEdgeCandidates.push_back(candidate.first);
#else
        m_activeEdgeCandidates.push_back(candidate.first);
#endif
        m_edgePointsCandidates.bitmap[candidate.first] = WEAK_EDGE;
      }
    }

#ifdef VISP_HAVE_OPENMP
#pragma omp critical
    {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      m_activeEdgeCandidates.insert(
        m_activeEdgeCandidates.end(),
        std::make_move_iterator(localMemoryEdgeCandidates.begin()),
        std::make_move_iterator(localMemoryEdgeCandidates.end())
      );
#else
      m_activeEdgeCandidates.insert(
        m_activeEdgeCandidates.end(),
        localMemoryEdgeCandidates.begin(),
        localMemoryEdgeCandidates.end()
      );
#endif
    }
  }
#endif
}

void
vpCannyEdgeDetection::performEdgeTracking()
{
  const unsigned char var_uc_255 = 255;
  const unsigned int nbCols = m_edgeMap.getCols();

  std::vector<unsigned int>::iterator it;
  std::vector<unsigned int>::iterator m_edgePointsCandidates_end = m_activeEdgeCandidates.end();
  for (it = m_activeEdgeCandidates.begin(); it != m_edgePointsCandidates_end; ++it) {
    if (m_edgePointsCandidates.bitmap[*it] == STRONG_EDGE) {
      if (m_storeListEdgePoints) {
        if (m_edgeMap.bitmap[*it] != var_uc_255) {
          // Edge point not added yet to the edge list
          unsigned int row = *it / nbCols;
          unsigned int col = *it % nbCols;
          m_edgePointsList.push_back(vpImagePoint(row, col));
        }
      }
      m_edgeMap.bitmap[*it] = var_uc_255;
    }
    else if (m_edgePointsCandidates.bitmap[*it] == WEAK_EDGE) {
      recursiveSearchForStrongEdge(*it);
    }
  }
}

bool
vpCannyEdgeDetection::recursiveSearchForStrongEdge(const unsigned int &coordinates)
{
  const int nbCols = static_cast<int>(m_dIx.getCols());
  const int size = static_cast<int>(m_dIx.getSize());
  const int coordAsInt = static_cast<int>(coordinates);
  bool hasFoundStrongEdge = false;
  m_edgePointsCandidates.bitmap[coordinates] = ON_CHECK;
  bool test_row = false;
  bool test_col = false;
  bool test_drdc = false;
  bool edge_in_image_limit = false;
  int dr = -1;
  while ((dr <= 1) && (!hasFoundStrongEdge)) {
    int dc = -1;
    while ((dc <= 1) && (!hasFoundStrongEdge)) {
      // reset the check for the edge on image limit
      edge_in_image_limit = false;

      int iterTest = coordAsInt + dr * nbCols + dc;

      // Checking if we are still looking for an edge in the limit of the image
      test_row = (iterTest < 0) || (iterTest >= size);
      test_col = ((iterTest - dc) / nbCols) != (iterTest / nbCols);
      test_drdc = (dr == 0) && (dc == 0);
      if (test_row || test_col || test_drdc) {
        edge_in_image_limit = true;
        // the continue is replaced by the test
      }
      if (edge_in_image_limit == false) {

        try {
          // Checking if the 8-neighbor point is in the list of edge candidates
          EdgeType type_candidate = m_edgePointsCandidates.bitmap[iterTest];
          if (type_candidate == STRONG_EDGE) {
            // The 8-neighbor point is a strong edge => the weak edge becomes a strong edge
            hasFoundStrongEdge = true;
          }
          else if (type_candidate == WEAK_EDGE) {
            // Checking if the WEAK_EDGE neighbor has a STRONG_EDGE neighbor
            hasFoundStrongEdge = recursiveSearchForStrongEdge(iterTest);
          }
        }
        catch (...) {
          // continue - nothing to do
        }
      }
      ++dc;
    }
    ++dr;
  }
  const unsigned char var_uc_255 = 255;
  if (hasFoundStrongEdge) {
    if (m_storeListEdgePoints) {
      if (m_edgeMap.bitmap[coordinates] != var_uc_255) {
        // Edge point not added yet to the edge list
        unsigned int row = coordinates / nbCols;
        unsigned int col = coordinates % nbCols;
        m_edgePointsList.push_back(vpImagePoint(row, col));
      }
    }
    m_edgePointsCandidates.bitmap[coordinates] = STRONG_EDGE;
    m_edgeMap.bitmap[coordinates] = var_uc_255;
  }
  return hasFoundStrongEdge;
}
END_VISP_NAMESPACE
