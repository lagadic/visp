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
*****************************************************************************/

#include <visp3/core/vpCannyEdgeDetection.h>

#include <visp3/core/vpImageConvert.h>

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
};
#endif

BEGIN_VISP_NAMESPACE
#ifdef VISP_HAVE_NLOHMANN_JSON
void from_json(const nlohmann::json &j, vpCannyEdgeDetection &detector)
{
  std::string filteringAndGradientName = vpImageFilter::vpCannyFiltAndGradTypeToStr(detector.m_filteringAndGradientType);
  filteringAndGradientName = j.value("filteringAndGradientType", filteringAndGradientName);
  detector.m_filteringAndGradientType = vpImageFilter::vpCannyFiltAndGradTypeFromStr(filteringAndGradientName);
  detector.m_gaussianKernelSize = j.value("gaussianSize", detector.m_gaussianKernelSize);
  detector.m_gaussianStdev = j.value("gaussianStdev", detector.m_gaussianStdev);
  detector.m_lowerThreshold = j.value("lowerThreshold", detector.m_lowerThreshold);
  detector.m_lowerThresholdRatio = j.value("lowerThresholdRatio", detector.m_lowerThresholdRatio);
  detector.m_gradientFilterKernelSize = j.value("gradientFilterKernelSize", detector.m_gradientFilterKernelSize);
  detector.m_upperThreshold = j.value("upperThreshold", detector.m_upperThreshold);
  detector.m_upperThresholdRatio = j.value("upperThresholdRatio", detector.m_upperThresholdRatio);
}

void to_json(nlohmann::json &j, const vpCannyEdgeDetection &detector)
{
  std::string filteringAndGradientName = vpImageFilter::vpCannyFiltAndGradTypeToStr(detector.m_filteringAndGradientType);
  j = nlohmann::json {
          {"filteringAndGradientType", filteringAndGradientName},
          {"gaussianSize", detector.m_gaussianKernelSize},
          {"gaussianStdev", detector.m_gaussianStdev},
          {"lowerThreshold", detector.m_lowerThreshold},
          {"lowerThresholdRatio", detector.m_lowerThresholdRatio},
          {"gradientFilterKernelSize", detector.m_gradientFilterKernelSize},
          {"upperThreshold", detector.m_upperThreshold},
          {"upperThresholdRatio", detector.m_upperThresholdRatio}
  };
}
#endif

// // Initialization methods

vpCannyEdgeDetection::vpCannyEdgeDetection()
  : m_filteringAndGradientType(vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
  , m_gaussianKernelSize(3)
  , m_gaussianStdev(1.f)
  , m_areGradientAvailable(false)
  , m_gradientFilterKernelSize(3)
  , m_lowerThreshold(-1.f)
  , m_lowerThresholdRatio(0.6f)
  , m_upperThreshold(-1.f)
  , m_upperThresholdRatio(0.8f)
  , mp_mask(nullptr)
{
  initGaussianFilters();
  initGradientFilters();
}

vpCannyEdgeDetection::vpCannyEdgeDetection(const int &gaussianKernelSize, const float &gaussianStdev
                                           , const unsigned int &sobelAperture, const float &lowerThreshold, const float &upperThreshold
                                           , const float &lowerThresholdRatio, const float &upperThresholdRatio
                                           , const vpImageFilter::vpCannyFilteringAndGradientType &filteringType
)
  : m_filteringAndGradientType(filteringType)
  , m_gaussianKernelSize(gaussianKernelSize)
  , m_gaussianStdev(gaussianStdev)
  , m_areGradientAvailable(false)
  , m_gradientFilterKernelSize(sobelAperture)
  , m_lowerThreshold(lowerThreshold)
  , m_lowerThresholdRatio(lowerThresholdRatio)
  , m_upperThreshold(upperThreshold)
  , m_upperThresholdRatio(upperThresholdRatio)
  , mp_mask(nullptr)
{
  initGaussianFilters();
  initGradientFilters();
}

#ifdef VISP_HAVE_NLOHMANN_JSON

using json = nlohmann::json;

vpCannyEdgeDetection::vpCannyEdgeDetection(const std::string &jsonPath)
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
  initGaussianFilters();
  initGradientFilters();
}
#endif

void
vpCannyEdgeDetection::initGaussianFilters()
{
  if ((m_gaussianKernelSize % 2) == 0) {
    throw(vpException(vpException::badValue, "The Gaussian kernel size should be odd"));
  }
  m_fg.resize(1, (m_gaussianKernelSize + 1) / 2);
  vpImageFilter::getGaussianKernel(m_fg.data, m_gaussianKernelSize, m_gaussianStdev, true);
}

void
vpCannyEdgeDetection::initGradientFilters()
{
  if ((m_gradientFilterKernelSize % 2) != 1) {
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
    scaleX = vpImageFilter::getSobelKernelX(m_gradientFilterX.data, (m_gradientFilterKernelSize - 1) / 2);
    scaleY = vpImageFilter::getSobelKernelY(m_gradientFilterY.data, (m_gradientFilterKernelSize - 1) / 2);
  }
  else if (m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING) {
    // Compute the Scharr filters
    scaleX = vpImageFilter::getScharrKernelX(m_gradientFilterX.data, (m_gradientFilterKernelSize - 1) / 2);
    scaleY = vpImageFilter::getScharrKernelY(m_gradientFilterY.data, (m_gradientFilterKernelSize - 1) / 2);
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
  // // Clearing the previous results
  m_edgeMap.resize(I.getHeight(), I.getWidth(), 0);
  m_edgeCandidateAndGradient.clear();
  m_edgePointsCandidates.clear();

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
  performEdgeThinning(lowerThreshold);

  // // Step 4: hysteresis thresholding
  performHysteresisThresholding(lowerThreshold, upperThreshold);

  // // Step 5: edge tracking
  performEdgeTracking();
  return m_edgeMap;
}

void
vpCannyEdgeDetection::computeFilteringAndGradient(const vpImage<unsigned char> &I)
{
  if ((m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
      || (m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING)) {
    // Computing the Gaussian blur
    vpImage<float> Iblur;
    vpImage<float> GIx;
    vpImageFilter::filterX<unsigned char, float>(I, GIx, m_fg.data, m_gaussianKernelSize, mp_mask);
    vpImageFilter::filterY<float, float>(GIx, Iblur, m_fg.data, m_gaussianKernelSize, mp_mask);

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
getInterpolWeightsAndOffsets(const float &gradientOrientation,
                                  float &alpha, float &beta,
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
    dRowGradBeta = -1;
  }
  else if ((gradientOrientation >= M_PI_4_FLOAT) && (gradientOrientation < M_PI_2_FLOAT)) {
    // Angles between 45 and 90 deg rely on the diagonal and vertical points
    thetaMin = M_PI_4_FLOAT;
    dColGradAlpha = 1;
    dColGradBeta = 0;
    dRowGradAlpha = -1;
    dRowGradBeta = -1;
  }
  else if ((gradientOrientation >= M_PI_2_FLOAT) && (gradientOrientation < (3.f * M_PI_4_FLOAT))) {
    // Angles between 90 and 135 deg rely on the vertical and diagonal points
    thetaMin = M_PI_2_FLOAT;
    dColGradAlpha = 0;
    dColGradBeta = -1;
    dRowGradAlpha = -1;
    dRowGradBeta = -1;
  }
  else if ((gradientOrientation >= (3.f * M_PI_4_FLOAT)) && (gradientOrientation < M_PI_FLOAT)) {
    // Angles between 135 and 180 deg rely on the vertical and diagonal points
    thetaMin = 3.f * M_PI_4_FLOAT;
    dColGradAlpha = -1;
    dColGradBeta = -1;
    dRowGradAlpha = -1;
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
getManhattanGradient(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &row, const int &col)
{
  float grad = 0.;
  int nbRows = dIx.getRows();
  int nbCols = dIx.getCols();
  if ((row >= 0)
      && (row < nbRows)
      && (col >= 0)
      && (col < nbCols)
      ) {
    float dx = dIx[row][col];
    float dy = dIy[row][col];
    grad = std::abs(dx) + std::abs(dy);
  }
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
getGradientOrientation(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &row, const int &col)
{
  float gradientOrientation = 0.f;
  float dx = dIx[row][col];
  float dy = dIy[row][col];

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
  int nbRows = m_dIx.getRows();
  int nbCols = m_dIx.getCols();

  bool ignore_current_pixel = false;
  bool grad_lower_threshold = false;
  for (int row = 0; row < nbRows; ++row) {
    for (int col = 0; col < nbCols; ++col) {
      // reset the checks
      ignore_current_pixel = false;
      grad_lower_threshold = false;

      if (mp_mask != nullptr) {
        if (!(*mp_mask)[row][col]) {
          // The mask tells us to ignore the current pixel
          ignore_current_pixel = true;
          // continue
        }
      }
      // continue if the mask does not tell us to ignore the current pixel
      if (ignore_current_pixel == false) {

        // Computing the gradient orientation and magnitude
        float grad = getManhattanGradient(m_dIx, m_dIy, row, col);

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
          float gradientOrientation = getGradientOrientation(m_dIx, m_dIy, row, col);
          float alpha = 0.f, beta = 0.f;
          getInterpolWeightsAndOffsets(gradientOrientation, alpha, beta, dRowAlphaPlus, dRowBetaPlus, dColAphaPlus, dColBetaPlus);
          int dRowAlphaMinus = -dRowAlphaPlus, dRowBetaMinus = -dRowBetaPlus;
          int dColAphaMinus = -dColAphaPlus, dColBetaMinus = -dColBetaPlus;
          float gradAlphaPlus = getManhattanGradient(m_dIx, m_dIy, row + dRowAlphaPlus, col + dColAphaPlus);
          float gradBetaPlus = getManhattanGradient(m_dIx, m_dIy, row + dRowBetaPlus, col + dColBetaPlus);
          float gradAlphaMinus = getManhattanGradient(m_dIx, m_dIy, row + dRowAlphaMinus, col + dColAphaMinus);
          float gradBetaMinus = getManhattanGradient(m_dIx, m_dIy, row + dRowBetaMinus, col + dColBetaMinus);
          float gradPlus = (alpha * gradAlphaPlus) + (beta * gradBetaPlus);
          float gradMinus = (alpha * gradAlphaMinus) + (beta * gradBetaMinus);

          if ((grad >= gradPlus) && (grad >= gradMinus)) {
            // Keeping the edge point that has the highest gradient
            std::pair<unsigned int, unsigned int> bestPixel(row, col);
            m_edgeCandidateAndGradient[bestPixel] = grad;
          }
        }
      }
    }
  }
}

void
vpCannyEdgeDetection::performHysteresisThresholding(const float &lowerThreshold, const float &upperThreshold)
{
  std::map<std::pair<unsigned int, unsigned int>, float>::iterator it;
  std::map<std::pair<unsigned int, unsigned int>, float>::iterator m_edgeCandidateAndGradient_end = m_edgeCandidateAndGradient.end();
  for (it = m_edgeCandidateAndGradient.begin(); it != m_edgeCandidateAndGradient_end; ++it) {
    if (it->second >= upperThreshold) {
      m_edgePointsCandidates[it->first] = STRONG_EDGE;
    }
    else if ((it->second >= lowerThreshold) && (it->second < upperThreshold)) {
      m_edgePointsCandidates[it->first] = WEAK_EDGE;
    }
  }
}

void
vpCannyEdgeDetection::performEdgeTracking()
{
  std::map<std::pair<unsigned int, unsigned int>, EdgeType>::iterator it;
  std::map<std::pair<unsigned int, unsigned int>, EdgeType>::iterator m_edgePointsCandidates_end = m_edgePointsCandidates.end();
  for (it = m_edgePointsCandidates.begin(); it != m_edgePointsCandidates_end; ++it) {
    if (it->second == STRONG_EDGE) {
      m_edgeMap[it->first.first][it->first.second] = 255;
    }
    else if (it->second == WEAK_EDGE) {
      if (recursiveSearchForStrongEdge(it->first)) {
        m_edgeMap[it->first.first][it->first.second] = 255;
      }
    }
  }
}

bool
vpCannyEdgeDetection::recursiveSearchForStrongEdge(const std::pair<unsigned int, unsigned int> &coordinates)
{
  bool hasFoundStrongEdge = false;
  int nbRows = m_dIx.getRows();
  int nbCols = m_dIx.getCols();
  m_edgePointsCandidates[coordinates] = ON_CHECK;
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

      int idRow = dr + static_cast<int>(coordinates.first);
      idRow = std::max<int>(idRow, 0); // Avoid getting negative pixel ID
      int idCol = dc + static_cast<int>(coordinates.second);
      idCol = std::max<int>(idCol, 0); // Avoid getting negative pixel ID

      // Checking if we are still looking for an edge in the limit of the image
      test_row = (idRow < 0) || (idRow >= nbRows);
      test_col = (idCol < 0) || (idCol >= nbCols);
      test_drdc = (dr == 0) && (dc == 0);
      if (test_row || test_col || test_drdc) {
        edge_in_image_limit = true;
        // the continue is replaced by the test
      }
      if (edge_in_image_limit == false) {

        try {
          std::pair<unsigned int, unsigned int> key_candidate(idRow, idCol);
          // Checking if the 8-neighbor point is in the list of edge candidates
          EdgeType type_candidate = m_edgePointsCandidates.at(key_candidate);
          if (type_candidate == STRONG_EDGE) {
            // The 8-neighbor point is a strong edge => the weak edge becomes a strong edge
            hasFoundStrongEdge = true;
          }
          else if (type_candidate == WEAK_EDGE) {
            // Checking if the WEAK_EDGE neighbor has a STRONG_EDGE neighbor
            hasFoundStrongEdge = recursiveSearchForStrongEdge(key_candidate);
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
  if (hasFoundStrongEdge) {
    m_edgePointsCandidates[coordinates] = STRONG_EDGE;
    m_edgeMap[coordinates.first][coordinates.second] = 255;
  }
  return hasFoundStrongEdge;
}
END_VISP_NAMESPACE
