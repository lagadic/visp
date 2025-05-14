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
 */

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageMorphology.h>

#include <visp3/imgproc/vpCircleHoughTransform.h>

BEGIN_VISP_NAMESPACE

#if (VISP_CXX_STANDARD == VISP_CXX_STANDARD_98)
namespace
{

// Sorting by decreasing probabilities
bool hasBetterProba(std::pair<size_t, float> a, std::pair<size_t, float> b)
{
  return (a.second > b.second);
}


void
scaleFilter(vpArray2D<float> &filter, const float &scale)
{
  unsigned int nbRows = filter.getRows();
  unsigned int nbCols = filter.getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      filter[r][c] = filter[r][c] * scale;
    }
  }
}
}
#endif

vpCircleHoughTransform::vpCircleHoughTransform()
  : m_algoParams()
  , mp_mask(nullptr)
{
  initGaussianFilters();
  initGradientFilters();
}

vpCircleHoughTransform::vpCircleHoughTransform(const vpCircleHoughTransformParams &algoParams)
  : m_algoParams(algoParams)
  , mp_mask(nullptr)
{
  initGaussianFilters();
  initGradientFilters();
}

void
vpCircleHoughTransform::init(const vpCircleHoughTransformParams &algoParams)
{
  m_algoParams = algoParams;
  initGaussianFilters();
  initGradientFilters();
}

vpCircleHoughTransform::~vpCircleHoughTransform()
{ }

#ifdef VISP_HAVE_NLOHMANN_JSON
using json = nlohmann::json;

vpCircleHoughTransform::vpCircleHoughTransform(const std::string &jsonPath)
  : mp_mask(nullptr)
{
  initFromJSON(jsonPath);
}

void
vpCircleHoughTransform::initFromJSON(const std::string &jsonPath)
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
  m_algoParams = j; // Call from_json(const json& j, vpDetectorDNN& *this) to read json
  file.close();
  initGaussianFilters();
  initGradientFilters();
}

void
vpCircleHoughTransform::saveConfigurationInJSON(const std::string &jsonPath) const
{
  m_algoParams.saveConfigurationInJSON(jsonPath);
}
#endif

void
vpCircleHoughTransform::initGaussianFilters()
{
  const int filterHalfSize = (m_algoParams.m_gaussianKernelSize + 1) / 2;
  m_fg.resize(1, filterHalfSize);
  vpImageFilter::getGaussianKernel(m_fg.data, m_algoParams.m_gaussianKernelSize, m_algoParams.m_gaussianStdev, true);
  m_cannyVisp.setGaussianFilterParameters(m_algoParams.m_gaussianKernelSize, m_algoParams.m_gaussianStdev);
}

void
vpCircleHoughTransform::initGradientFilters()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) // Check if cxx11 or higher
  // Helper to apply the scale to the raw values of the filters
  auto scaleFilter = [](vpArray2D<float> &filter, const float &scale) {
    const unsigned int nbRows = filter.getRows();
    const unsigned int nbCols = filter.getCols();
    for (unsigned int r = 0; r < nbRows; ++r) {
      for (unsigned int c = 0; c < nbCols; ++c) {
        filter[r][c] = filter[r][c] * scale;
      }
    }
    };
#endif

  const int moduloCheckForOddity = 2;
  if ((m_algoParams.m_gradientFilterKernelSize % moduloCheckForOddity) != 1) {
    throw vpException(vpException::badValue, "Gradient filters Kernel size should be odd.");
  }
  m_gradientFilterX.resize(m_algoParams.m_gradientFilterKernelSize, m_algoParams.m_gradientFilterKernelSize);
  m_gradientFilterY.resize(m_algoParams.m_gradientFilterKernelSize, m_algoParams.m_gradientFilterKernelSize);
  m_cannyVisp.setGradientFilterAperture(m_algoParams.m_gradientFilterKernelSize);

  float scaleX = 1.f;
  float scaleY = 1.f;
  unsigned int filterHalfSize = (m_algoParams.m_gradientFilterKernelSize - 1) / 2;

  if (m_algoParams.m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING) {
    // Compute the Sobel filters
    scaleX = vpImageFilter::getSobelKernelX(m_gradientFilterX.data, filterHalfSize);
    scaleY = vpImageFilter::getSobelKernelY(m_gradientFilterY.data, filterHalfSize);
  }
  else if (m_algoParams.m_filteringAndGradientType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING) {
    // Compute the Scharr filters
    scaleX = vpImageFilter::getScharrKernelX(m_gradientFilterX.data, filterHalfSize);
    scaleY = vpImageFilter::getScharrKernelY(m_gradientFilterY.data, filterHalfSize);
  }
  else {
    std::string errMsg = "[vpCircleHoughTransform::initGradientFilters] Error: gradient filtering method \"";
    errMsg += vpImageFilter::vpCannyFiltAndGradTypeToStr(m_algoParams.m_filteringAndGradientType);
    errMsg += "\" has not been implemented yet\n";
    throw vpException(vpException::notImplementedError, errMsg);
  }
  scaleFilter(m_gradientFilterX, scaleX);
  scaleFilter(m_gradientFilterY, scaleY);
}

std::vector<vpImageCircle>
vpCircleHoughTransform::detect(const vpImage<vpRGBa> &I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(I, I_gray);
  return detect(I_gray);
}

#ifdef HAVE_OPENCV_CORE
std::vector<vpImageCircle>
vpCircleHoughTransform::detect(const cv::Mat &cv_I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(cv_I, I_gray);
  return detect(I_gray);
}
#endif

std::vector<vpImageCircle>
vpCircleHoughTransform::detect(const vpImage<unsigned char> &I, const int &nbCircles)
{
  std::vector<vpImageCircle> detections = detect(I);
  size_t nbDetections = detections.size();

  // Prepare vector of tuple to sort by decreasing probabilities
  std::vector<std::pair<size_t, float> > v_id_proba;
  for (size_t i = 0; i < nbDetections; ++i) {
    std::pair<size_t, float> id_proba(i, m_finalCirclesProbabilities[i]);
    v_id_proba.push_back(id_proba);
  }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  // Sorting by decreasing probabilities
  auto hasBetterProba
    = [](std::pair<size_t, float> a, std::pair<size_t, float> b) {
    return (a.second > b.second);
    };
#endif
  std::sort(v_id_proba.begin(), v_id_proba.end(), hasBetterProba);

  // Clearing the storages containing the detection results
  // to have it sorted by decreasing probabilities
  size_t limitMin;
  if (nbCircles < 0) {
    limitMin = nbDetections;
  }
  else {
    limitMin = std::min(nbDetections, static_cast<size_t>(nbCircles));
  }

  std::vector<vpImageCircle> bestCircles;
  std::vector<vpImageCircle> copyFinalCircles = m_finalCircles;
  std::vector<unsigned int> copyFinalCirclesVotes = m_finalCircleVotes;
  std::vector<float> copyFinalCirclesProbas = m_finalCirclesProbabilities;
  std::vector<std::vector<std::pair<unsigned int, unsigned int> > > copyFinalCirclesVotingPoints = m_finalCirclesVotingPoints;
  for (size_t i = 0; i < nbDetections; ++i) {
    size_t id = v_id_proba[i].first;
    m_finalCircles[i] = copyFinalCircles[id];
    m_finalCircleVotes[i] = copyFinalCirclesVotes[id];
    m_finalCirclesProbabilities[i] = copyFinalCirclesProbas[id];
    if (m_algoParams.m_recordVotingPoints) {
      m_finalCirclesVotingPoints[i] = copyFinalCirclesVotingPoints[id];
    }
    if (i < limitMin) {
      bestCircles.push_back(m_finalCircles[i]);
    }
  }

  return bestCircles;
}

std::vector<vpImageCircle>
vpCircleHoughTransform::detect(const vpImage<unsigned char> &I)
{
  // Cleaning results of potential previous detection
  m_centerCandidatesList.clear();
  m_centerVotes.clear();
  m_edgePointsList.clear();
  m_circleCandidates.clear();
  m_circleCandidatesVotes.clear();
  m_circleCandidatesProbabilities.clear();
  m_circleCandidatesVotingPoints.clear();
  m_finalCircles.clear();
  m_finalCircleVotes.clear();
  m_finalCirclesProbabilities.clear();
  m_finalCirclesVotingPoints.clear();

  // Ensuring that the difference between the max and min radii is big enough to take into account
  // the pixelization of the image
  const float minRadiusDiff = 3.f;
  if ((m_algoParams.m_maxRadius - m_algoParams.m_minRadius) < minRadiusDiff) {
    if (m_algoParams.m_minRadius > (minRadiusDiff / 2.f)) {
      m_algoParams.m_maxRadius += minRadiusDiff / 2.f;
      m_algoParams.m_minRadius -= minRadiusDiff / 2.f;
    }
    else {
      m_algoParams.m_maxRadius += minRadiusDiff - m_algoParams.m_minRadius;
      m_algoParams.m_minRadius = 0.f;
    }
  }

  // Ensuring that the difference between the max and min center position is big enough to take into account
  // the pixelization of the image
  const float minCenterPositionDiff = 3.f;
  if ((m_algoParams.m_centerXlimits.second - m_algoParams.m_centerXlimits.first) < minCenterPositionDiff) {
    m_algoParams.m_centerXlimits.second += static_cast<int>(minCenterPositionDiff / 2.f);
    m_algoParams.m_centerXlimits.first -= static_cast<int>(minCenterPositionDiff / 2.f);
  }
  if ((m_algoParams.m_centerYlimits.second - m_algoParams.m_centerYlimits.first) < minCenterPositionDiff) {
    m_algoParams.m_centerYlimits.second += static_cast<int>(minCenterPositionDiff / 2.f);
    m_algoParams.m_centerYlimits.first -= static_cast<int>(minCenterPositionDiff / 2.f);
  }

  // First thing, we need to apply a Gaussian filter on the image to remove some spurious noise
  // Then, we need to compute the image gradients in order to be able to perform edge detection
  computeGradients(I);

  // Using the gradients, it is now possible to perform edge detection
  // We rely on the Canny edge detector
  // It will also give us the connected edged points
  edgeDetection(I);

  // From the edge map and gradient information, it is possible to compute
  // the center point candidates
  computeCenterCandidates();

  // From the edge map and center point candidates, we can compute candidate
  // circles. These candidate circles are circles whose center belong to
  // the center point candidates and whose radius is a "radius bin" that got
  // enough votes by computing the distance between each point of the edge map
  // and the center point candidate
  computeCircleCandidates();

  // Finally, we perform a merging operation that permits to merge circles
  // respecting similarity criteria (distance between centers and similar radius)
  mergeCircleCandidates();

  return m_finalCircles;
}

VISP_EXPORT bool operator==(const vpImageCircle &a, const vpImageCircle &b)
{
  vpImagePoint aCenter = a.getCenter();
  vpImagePoint bCenter = b.getCenter();
  bool haveSameCenter = (std::abs(aCenter.get_u() - bCenter.get_u())
                         + std::abs(aCenter.get_v() - bCenter.get_v())) <= (2. * std::numeric_limits<double>::epsilon());
  bool haveSameRadius = std::abs(a.getRadius() - b.getRadius()) <= (2.f * std::numeric_limits<float>::epsilon());
  return (haveSameCenter && haveSameRadius);
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
void vpCircleHoughTransform::computeVotingMask(const vpImage<unsigned char> &I, const std::vector<vpImageCircle> &detections,
                         std::optional< vpImage<bool> > &mask, std::optional<std::vector<std::vector<std::pair<unsigned int, unsigned int>>>> &opt_votingPoints) const
#else
void vpCircleHoughTransform::computeVotingMask(const vpImage<unsigned char> &I, const std::vector<vpImageCircle> &detections,
                         vpImage<bool> **mask, std::vector<std::vector<std::pair<unsigned int, unsigned int> > > **opt_votingPoints) const
#endif
{
  if (!m_algoParams.m_recordVotingPoints) {
    // We weren't asked to remember the voting points
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
    mask = std::nullopt;
    opt_votingPoints = std::nullopt;
#else
    *mask = nullptr;
    *opt_votingPoints = nullptr;
#endif
    return;
  }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  mask = vpImage<bool>(I.getHeight(), I.getWidth(), false);
  opt_votingPoints = std::vector<std::vector<std::pair<unsigned int, unsigned int>>>();
#else
  *mask = new vpImage<bool>(I.getHeight(), I.getWidth(), false);
  *opt_votingPoints = new std::vector<std::vector<std::pair<unsigned int, unsigned int> > >();
#endif

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  for (const auto &detection : detections)
#else
  const size_t nbDetections = detections.size();
  for (size_t i = 0; i < nbDetections; ++i)
#endif
  {
    bool hasFoundSimilarCircle = false;
    unsigned int nbPreviouslyDetected = static_cast<unsigned int>(m_finalCircles.size());
    unsigned int id = 0;
    // Looking for a circle that was detected and is similar to the one given to the function
    while ((id < nbPreviouslyDetected) && (!hasFoundSimilarCircle)) {
      vpImageCircle previouslyDetected = m_finalCircles[id];
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      if (previouslyDetected == detection)
#else
      if (previouslyDetected == detections[i])
#endif
      {
        hasFoundSimilarCircle = true;
        // We found a circle that is similar to the one given to the function => updating the mask
        const unsigned int nbVotingPoints = static_cast<unsigned int>(m_finalCirclesVotingPoints[id].size());
        for (unsigned int idPoint = 0; idPoint < nbVotingPoints; ++idPoint) {
          const std::pair<unsigned int, unsigned int> &votingPoint = m_finalCirclesVotingPoints[id][idPoint];
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
          (*mask)[votingPoint.first][votingPoint.second] = true;
#else
          (**mask)[votingPoint.first][votingPoint.second] = true;
#endif
        }
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
        opt_votingPoints->push_back(m_finalCirclesVotingPoints[id]);
#else
        (**opt_votingPoints).push_back(m_finalCirclesVotingPoints[id]);
#endif
      }
      ++id;
    }
  }
}

std::string
vpCircleHoughTransform::toString() const
{
  return m_algoParams.toString();
}

std::ostream &
operator<<(std::ostream &os, const vpCircleHoughTransform &detector)
{
  os << detector.toString();
  std::cout << "\tUse mask: " << (detector.mp_mask == nullptr ? "false" : "true") << std::endl;
  return os;
}

END_VISP_NAMESPACE
