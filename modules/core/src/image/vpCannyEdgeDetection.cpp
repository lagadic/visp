/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

// // Initialization methods

vpCannyEdgeDetection::vpCannyEdgeDetection()
  : m_gaussianKernelSize(3)
  , m_gaussianStdev(1.)
  , m_areGradientAvailable(false)
  , m_lowerThreshold(-1.)
  , m_upperThreshold(-1.)
{
  initGaussianFilters();
}

vpCannyEdgeDetection::vpCannyEdgeDetection(const int &gaussianKernelSize, const float &gaussianStdev
                        , const float &lowerThreshold, const float &upperThreshold)
  : m_gaussianKernelSize(gaussianKernelSize)
  , m_gaussianStdev(gaussianStdev)
  , m_areGradientAvailable(false)
  , m_lowerThreshold(lowerThreshold)
  , m_upperThreshold(upperThreshold)
{
  initGaussianFilters();
}

#ifdef VISP_HAVE_NLOHMANN_JSON
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
  *this = j; // Call from_json(const json& j, vpDetectionCircle2D& *this) to read json
  file.close();
  initGaussianFilters();
}
#endif

void
vpCannyEdgeDetection::initGaussianFilters()
{
  if ((m_gaussianKernelSize % 2) == 0) {
    throw(vpException(vpException::badValue, "The Gaussian kernel size should be odd"));
  }
  m_fg.resize(1, (m_gaussianKernelSize + 1)/2);
  vpImageFilter::getGaussianKernel(m_fg.data, m_gaussianKernelSize, m_gaussianStdev, false);
  m_fgDg.resize(1, (m_gaussianKernelSize + 1)/2);
  vpImageFilter::getGaussianDerivativeKernel(m_fgDg.data, m_gaussianKernelSize, m_gaussianStdev, false);
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
    performFilteringAndGradientComputation(I);
  }
  m_areGradientAvailable = false; // Reset for next call

  // // Step 3: edge thining
  performEdgeThining();

  // // Step 4: hysteresis thresholding
  float upperThreshold = m_upperThreshold;

  float lowerThreshold = m_lowerThreshold;
  if (m_lowerThreshold < 0) {
    // Applying Canny recommendation to have the upper threshold 3 times greater than the lower threshold.
    lowerThreshold = m_upperThreshold / 3.;
  }

  performHysteresisThresholding(lowerThreshold, upperThreshold);

  // // Step 5: edge tracking
  performEdgeTracking();
  return m_edgeMap;
}

void
vpCannyEdgeDetection::performFilteringAndGradientComputation(const vpImage<unsigned char> &I)
{
  vpImageFilter::getGradXGauss2D(I,
    m_dIx,
    m_fg.data,
    m_fgDg.data,
    m_gaussianKernelSize
  );
  vpImageFilter::getGradYGauss2D(I,
    m_dIy,
    m_fg.data,
    m_fgDg.data,
    m_gaussianKernelSize
  );
}

/**
 * \brief Get the theta quadrant in which lies absoluteTheta and the offset along the horizontal
 * and vertical direction where to look for the neighbors.
 *
 * \param[in] absoluteTheta : The absolute value of the angle of the edge, expressed in degrees.
 * \param[out] dRowGradPlus : The offset in the vertical positive direction.
 * \param[out] dRowGradMinus : The offset in the vertical negative direction.
 * \param[out] dColGradPlus : The offset in the horizontal positive direction.
 * \param[out] dColGradMinus : The offset in the horizontal negative direction.
 * \return The quadrant in which lies the angle of the edge, expressed in degrees.
 */
int
getThetaQuadrant(const float &absoluteTheta, int &dRowGradPlus, int &dRowGradMinus, int &dColGradPlus, int &dColGradMinus)
{
  if (absoluteTheta < 22.5) {
    // Angles between -22.5 and 22.5 are mapped to be horizontal axis
    dColGradMinus = -1;
    dColGradPlus = 1;
    dRowGradPlus = dRowGradMinus = 0;
    return 0;
  }
  else if (absoluteTheta >= 22.5 && absoluteTheta < 67.5) {
    // Angles between 22.5 and 67.5 are mapped to the diagonal 45degree
    dRowGradMinus = dColGradMinus = -1;
    dRowGradPlus = dColGradPlus = 1;
    return 45;
  }
  else if (absoluteTheta >= 67.5 && absoluteTheta < 112.5) {
    // Angles between 67.5 and 112.5 are mapped to the vertical axis
    dColGradMinus = dColGradPlus = 0;
    dRowGradMinus = -1;
    dRowGradPlus = 1;
    return 90;
  }
  else if (absoluteTheta >= 112.5 && absoluteTheta < 157.5) {
    // Angles between 112.5 and 157.5 are mapped to the diagonal -45degree
    dRowGradMinus = -1;
    dColGradMinus = 1;
    dRowGradPlus = 1;
    dColGradPlus = -1;
    return 135;
  }
  else {
    // Angles greater than 157.5 are mapped to be horizontal axis
    dColGradMinus = 1;
    dColGradPlus = -1;
    dRowGradMinus = dRowGradPlus = 0;
    return 180;
  }
  return -1; // Should not reach this point
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
  if (row  >= 0
    && row  < nbRows
    && col  >= 0
    && col  < nbCols
    ) {
    float dx = dIx[row][col];
    float dy = dIy[row][col];
    grad = std::abs(dx) + std::abs(dy);
  }
  return grad;
}

/**
 * @brief Get the absolute value of the gradient orientation.
 *
 * @param dIx : Gradient along the horizontal axis.
 * @param dIy : Gradient along the vertical axis.
 * @param row : Index along the vertical axis.
 * @param col : Index along the horizontal axis.
 * @return float The absolute value of the gradient orientation, expressed in degrees.
 */
float
getAbsoluteTheta(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &row, const int &col)
{
  float absoluteTheta;
  float dx = dIx[row][col];
  float dy = dIy[row][col];

  if (std::abs(dx) < std::numeric_limits<float>::epsilon()) {
    absoluteTheta = 90.;
  }
  else {
    absoluteTheta = vpMath::deg(std::abs(std::atan(dy / dx)));
  }
  return absoluteTheta;
}
void
vpCannyEdgeDetection::performEdgeThining()
{
  vpImage<float> dIx = m_dIx;
  vpImage<float> dIy = m_dIy;
  int nbRows = m_dIx.getRows();
  int nbCols = m_dIx.getCols();

  for (int row = 0; row < nbRows; row++) {
    for (int col = 0; col < nbCols; col++) {
      // Computing the gradient orientation and magnitude
      float grad = getManhattanGradient(dIx, dIy, row, col);

      if (grad < std::numeric_limits<float>::epsilon()) {
        // The gradient is almost null => ignoring the point
        continue;
      }

      float absoluteTheta = getAbsoluteTheta(dIx, dIy, row, col);

      // Getting the offset along the horizontal and vertical axes
      // depending on the gradient orientation
      int dRowGradPlus = 0, dRowGradMinus = 0;
      int dColGradPlus = 0, dColGradMinus = 0;
      int thetaQuadrant = getThetaQuadrant(absoluteTheta, dRowGradPlus, dRowGradMinus, dColGradPlus, dColGradMinus);

      bool isGradientInTheSameDirection = true;
      std::vector<std::pair<int, int>> pixelsSeen;
      std::pair<int, int> bestPixel(row, col);
      float bestGrad = grad;
      int rowCandidate = row + dRowGradPlus;
      int colCandidate = col + dColGradPlus;

      while (isGradientInTheSameDirection) {
        // Getting the gradients around the edge point
        float gradPlus = getManhattanGradient(dIx, dIy, rowCandidate, colCandidate);
        if (std::abs(gradPlus) < std::numeric_limits<float>::epsilon()) {
          // The gradient is almost null => ignoring the point
          isGradientInTheSameDirection = false;
          break;
        }
        int dRowGradPlusCandidate = 0, dRowGradMinusCandidate = 0;
        int dColGradPlusCandidate = 0, dColGradMinusCandidate = 0;
        float absThetaPlus = getAbsoluteTheta(dIx, dIy, rowCandidate, colCandidate);
        int thetaQuadrantCandidate = getThetaQuadrant(absThetaPlus, dRowGradPlusCandidate, dRowGradMinusCandidate, dColGradPlusCandidate, dColGradMinusCandidate);
        if (thetaQuadrantCandidate != thetaQuadrant) {
          isGradientInTheSameDirection = false;
          break;
        }

        std::pair<int, int> pixelCandidate(rowCandidate, colCandidate);
        if (gradPlus > bestGrad) {
          // The gradient is higher with the nex pixel candidate
          // Saving it
          bestGrad = gradPlus;
          pixelsSeen.push_back(bestPixel);
          bestPixel = pixelCandidate;
        }
        else {
          // Best pixel is still the best
          pixelsSeen.push_back(pixelCandidate);
        }
        rowCandidate += dRowGradPlus;
        colCandidate += dColGradPlus;
      }

      // Keeping the edge point that has the highest gradient
      m_edgeCandidateAndGradient[bestPixel] = bestGrad;

      // Suppressing non-maximum gradient
      for (std::vector<std::pair<int, int>>::iterator it = pixelsSeen.begin(); it != pixelsSeen.end(); it++) {
        // Suppressing non-maximum gradient
        int row_temp = it->first;
        int col_temp = it->second;
        dIx[row_temp][col_temp] = 0.;
        dIy[row_temp][col_temp] = 0.;
      }
    }
  }
}

void
vpCannyEdgeDetection::performHysteresisThresholding(const float &lowerThreshold, const float &upperThreshold)
{
  std::map<std::pair<unsigned int, unsigned int>, float>::iterator it;
  for (it = m_edgeCandidateAndGradient.begin(); it != m_edgeCandidateAndGradient.end(); it++) {
    if (it->second >= upperThreshold) {
      m_edgePointsCandidates[it->first] = STRONG_EDGE;
    }
    else if (it->second >= lowerThreshold && it->second < upperThreshold) {
      m_edgePointsCandidates[it->first] = WEAK_EDGE;
    }
  }
}

void
vpCannyEdgeDetection::performEdgeTracking()
{
  std::map<std::pair<unsigned int, unsigned int>, EdgeType>::iterator it;
  for (it = m_edgePointsCandidates.begin(); it != m_edgePointsCandidates.end(); it++) {
    if (it->second == STRONG_EDGE) {
      m_edgeMap[it->first.first][it->first.second] = 255;
    }
    else if (recursiveSearchForStrongEdge(it->first)) {
      m_edgeMap[it->first.first][it->first.second] = 255;
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
  for (int dr = -1; dr <= 1 && !hasFoundStrongEdge; dr++) {
    for (int dc = -1; dc <= 1 && !hasFoundStrongEdge; dc++) {
      int idRow = dr + (int)coordinates.first;
      int idCol = dc + (int)coordinates.second;

      // Checking if we are still looking for an edge in the limit of the image
      if ((idRow < 0 || idRow >= nbRows)
        || (idCol < 0 || idCol >= nbCols)
        || (dr == 0 && dc == 0)
        ) {
        continue;
      }

      try {
        std::pair<unsigned int, unsigned int> key_candidate(idRow, idCol);
        // Checking if the 8-neighbor point is in the list of edge candidates
        EdgeType type_candidate = m_edgePointsCandidates.at(key_candidate);
        if (type_candidate == STRONG_EDGE) {
          // The 8-neighbor point is a strong edge => the weak edge becomes a strong edge
          hasFoundStrongEdge = true;
        }
        else if (type_candidate == WEAK_EDGE) {
          hasFoundStrongEdge = recursiveSearchForStrongEdge(key_candidate);
        }
      }
      catch (std::out_of_range &e) {
        continue;
      }
    }
  }
  if (hasFoundStrongEdge) {
    m_edgePointsCandidates[coordinates] = STRONG_EDGE;
    m_edgeMap[coordinates.first][coordinates.second] = 255;
  }
  return hasFoundStrongEdge;
}
