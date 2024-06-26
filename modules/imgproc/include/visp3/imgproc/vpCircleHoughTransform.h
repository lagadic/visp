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

#ifndef VP_CIRCLE_HOUGH_TRANSFORM_H
#define VP_CIRCLE_HOUGH_TRANSFORM_H

// System includes
#include <utility>
#include <vector>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageCircle.h>
#include <visp3/core/vpMatrix.h>

// 3rd parties inclue
#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
#include <optional>
#endif

BEGIN_VISP_NAMESPACE

/**
 * \ingroup group_hough_transform
 * \brief Class that permits to detect 2D circles in a image using
 * the gradient-based Circle Hough transform.
 * Please find more information on the algorithm
 * [here](https://theailearner.com/tag/hough-gradient-method-opencv/)
 *
*/
class VISP_EXPORT vpCircleHoughTransform
{
public:
  /**
   * \brief Class that gather the algorithm parameters.
   */
  class vpCircleHoughTransformParams
  {
  public:
    /**
     * \brief Construct a new vpCircleHoughTransformParams object with default parameters.
     */
    vpCircleHoughTransformParams()
      : m_filteringAndGradientType(vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
      , m_gaussianStdev(1.f)
      , m_lowerCannyThresh(-1.f)
      , m_upperCannyThresh(-1.f)
      , m_edgeMapFilteringNbIter(1)
      , m_cannyBackendType(vpImageFilter::CANNY_OPENCV_BACKEND)
      , m_lowerCannyThreshRatio(0.6f)
      , m_upperCannyThreshRatio(0.8f)
      , m_centerXlimits(std::pair<int, int>(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()))
      , m_centerYlimits(std::pair<int, int>(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()))
      , m_minRadius(0.f)
      , m_maxRadius(1000.f)
      , m_centerMinThresh(50.f)
      , m_expectedNbCenters(-1)
      , m_circleProbaThresh(0.9f)
      , m_circlePerfectness(0.9f)
      , m_circleVisibilityRatioThresh(0.1f)
      , m_recordVotingPoints(false)
      , m_centerMinDist(15.f)
      , m_mergingRadiusDiffThresh(1.5f * m_centerMinDist)
    {
      const unsigned int gaussianKernelSize_default = 5;
      const unsigned int gradientFilterKernelSize_default = 3;
      const unsigned int dilatationKernelSize_default = 3;
      const unsigned int averagingWindowSize_default = 5;

      m_gaussianKernelSize = gaussianKernelSize_default;
      m_gradientFilterKernelSize = gradientFilterKernelSize_default;
      m_dilatationKernelSize = dilatationKernelSize_default;
      m_averagingWindowSize = averagingWindowSize_default;
    }

    /**
     * \brief Construct a new vpCircleHoughTransformParams object.
     *
     * \param[in] gaussianKernelSize Size of the Gaussian filter kernel used to smooth the input image. Must be an odd number.
     * \param[in] gaussianStdev Standard deviation of the Gaussian filter.
     * \param[in] gradientFilterKernelSize Size of the Sobel or Scharr kernels used to compute the gradients. Must be an odd number.
     * \param[in] lowerCannyThresh The lower threshold for the Canny operator. Values lower than this value are rejected.
                          A negative value makes the algorithm compute this threshold and the lower one automatically.
     * \param[in] upperCannyThresh The upper threshold for the Canny operator. Only values greater than this value are marked as an edge.
                          A negative value makes the algorithm compute this threshold and the lower one automatically.
     * \param[in] edgeMapFilterNbIter Number of 8-neighbor connectivity filtering iterations to apply to the edge map.
     * \param[in] centerXlimits Minimum and maximum position on the horizontal axis of the center of the circle we want to detect.
     * \param[in] centerYlimits Minimum and maximum position on the vertical axis of the center of the circle we want to detect.
     * \param[in] minRadius Minimum radius of the circles we want to detect.
     * \param[in] maxRadius Maximum radius of the circles we want to detect.
     * \param[in] dilatationKernelSize Kernel size of the dilatation that is performed to detect the maximum number of votes for the center candidates.
     * \param[in] averagingWindowSize Size of the averaging window around the maximum number of votes to compute the
                                      center candidate such as it is the barycenter of the window. Must be odd.
     * \param[in] centerThresh Minimum number of votes a point must exceed to be considered as center candidate.
     * \param[in] circleProbabilityThresh Probability threshold in order to keep a circle candidate.
     * \param[in] circlePerfectness The threshold for the colinearity between the gradient of a point
                                    and the radius it would form with a center candidate to be able to vote.
                                    The formula to get the equivalent angle is: `angle = acos(circle_perfectness)`.
     * \param[in] centerMinDistThresh  Two circle candidates whose centers are closer than this threshold are considered for merging.
     * \param[in] mergingRadiusDiffThresh Maximum radius difference between two circle candidates to consider merging them.
     * \param[in] filteringAndGradientMethod The choice of the filter and gradient operator to apply before the edge
     * detection step.
     * \param[in] backendType Permits to choose the backend used to compute the edge map.
     * \param[in] lowerCannyThreshRatio If the thresholds must be computed,the lower threshold will be equal to the upper
     * threshold times \b lowerThresholdRatio .
     * \param[in] upperCannyThreshRatio If the thresholds must be computed,the upper threshold will be equal to the value
     * such as the number of pixels of the image times \b upperThresholdRatio have an absolute gradient lower than the
     * upper threshold.
     * \param[in] expectedNbCenters Expected number of centers in the image. If the number is negative, all the centers
     * are kept. Otherwise, maximum up to this number of centers are kept.
     * \param[in] recordVotingPoints If true, the edge-map points having voted for each circle will be stored.
     * \param[in] visibilityRatioThresh Visibility threshold: which minimum ratio of the circle must be visible in order to keep a circle candidate.
     */
    vpCircleHoughTransformParams(
      const int &gaussianKernelSize
      , const float &gaussianStdev
      , const int &gradientFilterKernelSize
      , const float &lowerCannyThresh
      , const float &upperCannyThresh
      , const int &edgeMapFilterNbIter
      , const std::pair<int, int> &centerXlimits
      , const std::pair<int, int> &centerYlimits
      , const float &minRadius
      , const float &maxRadius
      , const int &dilatationKernelSize
      , const int &averagingWindowSize
      , const float &centerThresh
      , const float &circleProbabilityThresh
      , const float &circlePerfectness
      , const float &centerMinDistThresh
      , const float &mergingRadiusDiffThresh
      , const vpImageFilter::vpCannyFilteringAndGradientType &filteringAndGradientMethod = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING
      , const vpImageFilter::vpCannyBackendType &backendType = vpImageFilter::CANNY_OPENCV_BACKEND
      , const float &lowerCannyThreshRatio = 0.6f
      , const float &upperCannyThreshRatio = 0.8f
      , const int &expectedNbCenters = -1
      , const bool &recordVotingPoints = false
      , const float &visibilityRatioThresh = 0.1f
    )
      : m_filteringAndGradientType(filteringAndGradientMethod)
      , m_gaussianKernelSize(gaussianKernelSize)
      , m_gaussianStdev(gaussianStdev)
      , m_gradientFilterKernelSize(gradientFilterKernelSize)
      , m_lowerCannyThresh(lowerCannyThresh)
      , m_upperCannyThresh(upperCannyThresh)
      , m_edgeMapFilteringNbIter(edgeMapFilterNbIter)
      , m_cannyBackendType(backendType)
      , m_lowerCannyThreshRatio(lowerCannyThreshRatio)
      , m_upperCannyThreshRatio(upperCannyThreshRatio)
      , m_centerXlimits(centerXlimits)
      , m_centerYlimits(centerYlimits)
      , m_minRadius(std::min<float>(minRadius, maxRadius))
      , m_maxRadius(std::max<float>(minRadius, maxRadius))
      , m_dilatationKernelSize(dilatationKernelSize)
      , m_averagingWindowSize(averagingWindowSize)
      , m_centerMinThresh(centerThresh)
      , m_expectedNbCenters(expectedNbCenters)
      , m_circleProbaThresh(circleProbabilityThresh)
      , m_circlePerfectness(circlePerfectness)
      , m_circleVisibilityRatioThresh(visibilityRatioThresh)
      , m_recordVotingPoints(recordVotingPoints)
      , m_centerMinDist(centerMinDistThresh)
      , m_mergingRadiusDiffThresh(mergingRadiusDiffThresh)
    { }

    /**
     * \brief Get the size of the Gaussian filter kernel used to smooth the input image.
     *
     * \return int The size of the kernel.
     */
    inline int getGaussianKernelSize() const
    {
      return m_gaussianKernelSize;
    }

    /**
     * \brief Get the standard deviation of the Gaussian filter.
     *
     * \return float The standard deviation.
     */
    inline float getGaussianStdev() const
    {
      return m_gaussianStdev;
    }

    /**
     * \brief Get the size of the gradient kernel filters used to compute the gradients.
     *
     * \return int The size of the kernel.
     */
    inline int getGradientKernelSize() const
    {
      return m_gradientFilterKernelSize;
    }

    /**
     * \brief Get the lower threshold for the Canny operator. Values lower than this value are rejected.
     * A negative value means that the algorithm computes the lower threshold automatically.
     *
     * \return float The lower Canny threshold.
     */
    inline float getLowerCannyThreshold() const
    {
      return m_lowerCannyThresh;
    }

    /**
     * \brief Get the upper threshold for the Canny operator. Values lower than this value are rejected.
     * A negative value means that the algorithm computes the lower and upper thresholds automatically.
     *
     * \return float The upper Canny threshold.
     */
    inline float getUpperCannyThreshold() const
    {
      return m_upperCannyThresh;
    }

    /**
     * \brief Get the number of iterations of 8-neighbor connectivity filtering to apply to the edge map.
     *
     * \return int The number of iterations.
     */
    inline int getEdgeMapFilteringNbIter() const
    {
      return m_edgeMapFilteringNbIter;
    }

    /**
     * \brief Get the minimum and maximum position on the horizontal axis of the center of the circle we want to detect.
     *
     * \return std::pair<int, int> The min and max x positions.
     */
    inline std::pair<int, int> getCenterXLimits() const
    {
      return m_centerXlimits;
    }

    /**
     * \brief Get the minimum and maximum position on the vertical axis of the center of the circle we want to detect.
     *
     * \return std::pair<int, int> The min and max y positions.
     */
    inline std::pair<int, int> getCenterYLimits() const
    {
      return m_centerYlimits;
    }

    /**
     * \brief Get the minimum radius of the circles we want to detect.
     *
     * \return float The radius min.
     */
    inline float getMinRadius() const
    {
      return m_minRadius;
    }

    /**
     * \brief Get the maximum radius of the circles we want to detect.
     *
     * \return float The radius max.
     */
    inline float getMaxRadius() const
    {
      return m_maxRadius;
    }

    /**
     * \brief Get the kernel size of the dilatation that is performed to detect the maximum number of votes
     * for the center candidates.
     *
     * \return int The kernel size.
     */
    inline int getDilatationKernelSize() const
    {
      return m_dilatationKernelSize;
    }

    /**
     * \brief Get the size of the averaging window around the maximum number of votes to compute the
     * center candidate such as it is the barycenter of the window.
     *
     * \return int The size of the averaging window.
     */
    inline int getAveragingWindowSize() const
    {
      return m_averagingWindowSize;
    }

    /**
     * \brief Get the minimum number of votes a point must exceed to be considered as center candidate.
     *
     * \return float The threshold.
     */
    inline float getCenterMinThreshold() const
    {
      return m_centerMinThresh;
    }

    /**
     * \brief Get the expected number of centers in the image. If the number is negative, all the centers
     * are kept. Otherwise, maximum up to this number of centers are kept.
     *
     * \return int The expected number of centers.
     */
    inline int getExpectedNbCenters() const
    {
      return m_expectedNbCenters;
    }

    /**
     * \brief Get the probability threshold in order to keep a circle candidate.
     *
     * \return float The threshold.
     */
    inline float getProbabilityThreshold() const
    {
      return m_circleProbaThresh;
    }

    /**
     * \brief Get the visibility ratio threshold in order to keep a circle candidate.
     *
     * \return float The threshold.
     */
    inline float getVisibilityRatioThreshold() const
    {
      return m_circleVisibilityRatioThresh;
    }

    /**
     * \brief Get the threshold for the colinearity between the gradient of a point
     * and the radius it would form with a center candidate to be able to vote.
     * The formula to get the equivalent angle is: `angle = acos(circle_perfectness)`.
     *
     * \return float The threshold.
     */
    inline float getCirclePerfectness() const
    {
      return m_circlePerfectness;
    }

    /**
     * \brief Get the boolean indicating if we have to record the edge-map points having voted for the circles.
     *
     * \return bool True if we have to record the voting points.
     */
    inline bool getRecordVotingPoints() const
    {
      return m_recordVotingPoints;
    }

    /**
     * \brief Get the Maximum distance between two circle candidates centers to consider merging them.
     *
     * \return float The maximum distance between two centers.
     */
    inline float getCenterMinDist() const
    {
      return m_centerMinDist;
    }

    /**
     * \brief Get the Maximum radius difference between two circle candidates to consider merging them.
     *
     * @return float The merging radius difference.
     */
    inline float getMergingRadiusDiff() const
    {
      return m_mergingRadiusDiffThresh;
    }

    /**
     * Create a string with all the Hough transform parameters.
     */
    std::string toString() const
    {
      std::stringstream txt;
      txt << "Hough Circle Transform Configuration:\n";
      txt <<  "\tFiltering + gradient operators = " << vpImageFilter::vpCannyFiltAndGradTypeToStr(m_filteringAndGradientType) << "\n";
      txt <<  "\tGaussian filter kernel size = " << m_gaussianKernelSize << "\n";
      txt <<  "\tGaussian filter standard deviation = " << m_gaussianStdev << "\n";
      txt <<  "\tGradient filter kernel size = " << m_gradientFilterKernelSize << "\n";
      txt <<  "\tCanny backend = " << vpImageFilter::vpCannyBackendTypeToString(m_cannyBackendType) << "\n";
      txt <<  "\tCanny edge filter thresholds = [" << m_lowerCannyThresh << " ; " << m_upperCannyThresh << "]\n";
      txt <<  "\tCanny edge filter thresholds ratio (for auto-thresholding) = [" << m_lowerCannyThreshRatio << " ; " << m_upperCannyThreshRatio << "]\n";
      txt <<  "\tEdge map 8-neighbor connectivity filtering number of iterations = " << m_edgeMapFilteringNbIter << "\n";
      txt <<  "\tCenter horizontal position limits: min = " << m_centerXlimits.first << "\tmax = " << m_centerXlimits.second << "\n";
      txt <<  "\tCenter vertical position limits: min = " << m_centerYlimits.first << "\tmax = " << m_centerYlimits.second << "\n";
      txt <<  "\tRadius limits: min = " << m_minRadius << "\tmax = " << m_maxRadius << "\n";
      txt <<  "\tKernel size of the dilatation filter = " << m_dilatationKernelSize << "\n";
      txt <<  "\tAveraging window size for center detection = " << m_averagingWindowSize << "\n";
      txt <<  "\tCenters votes threshold = " << m_centerMinThresh << "\n";
      txt <<  "\tExpected number of centers = ";
      if (m_expectedNbCenters > 0) {
        txt << m_expectedNbCenters;
      }
      else {
        txt << "no limits";
      }
      txt << "\n";
      txt <<  "\tCircle probability threshold = " << m_circleProbaThresh << "\n";
      txt <<  "\tCircle visibility ratio threshold = " << m_circleVisibilityRatioThresh << "\n";
      txt <<  "\tCircle perfectness threshold = " << m_circlePerfectness << "\n";
      txt <<  "\tRecord voting points = ";
      txt << (m_recordVotingPoints ? std::string("true") : std::string("false")) << "\n";
      txt <<  "\tCenters minimum distance = " << m_centerMinDist << "\n";
      txt <<  "\tRadius difference merging threshold = " << m_mergingRadiusDiffThresh << "\n";
      return txt.str();
    }

    // // Configuration from files
#ifdef VISP_HAVE_NLOHMANN_JSON
  /**
   * \brief Create a new vpCircleHoughTransformParams from a JSON file.
   *
   * \param[in] jsonFile The path towards the JSON file.
   * \return vpCircleHoughTransformParams The corresponding vpCircleHoughTransformParams object.
   */
    inline static vpCircleHoughTransformParams createFromJSON(const std::string &jsonFile)
    {
      using json = nlohmann::json;

      std::ifstream file(jsonFile);
      if (!file.good()) {
        std::stringstream ss;
        ss << "Problem opening file " << jsonFile << ". Make sure it exists and is readable" << std::endl;
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
      vpCircleHoughTransformParams params = j; // Call from_json(const json& j, vpDetectorDNN& *this) to read json
      file.close();
      return params;
    }

    /**
     * \brief Save the configuration of the detector in a JSON file
     * described by the path \b jsonPath. Throw a \b vpException
     * is the file cannot be created.
     *
     * \param[in] jsonPath The path towards the JSON output file.
     */
    inline void saveConfigurationInJSON(const std::string &jsonPath) const
    {
      using json = nlohmann::json;
      std::ofstream file(jsonPath);
      const json j = *this;
      const int indent = 4;
      file << j.dump(indent);
      file.close();
    }

    /**
     * \brief Read the detector configuration from JSON. All values are optional and if an argument is not present,
     * the default value defined in the constructor is kept
     *
     * \param[in] j : The JSON object, resulting from the parsing of a JSON file.
     * \param[out] params : The circle Hough transform parameters that will be initialized from the JSON data.
     */
    friend inline void from_json(const nlohmann::json &j, vpCircleHoughTransformParams &params)
    {
      std::string filteringAndGradientName = vpImageFilter::vpCannyFiltAndGradTypeToStr(params.m_filteringAndGradientType);
      filteringAndGradientName = j.value("filteringAndGradientType", filteringAndGradientName);
      params.m_filteringAndGradientType = vpImageFilter::vpCannyFiltAndGradTypeFromStr(filteringAndGradientName);

      params.m_gaussianKernelSize = j.value("gaussianKernelSize", params.m_gaussianKernelSize);
      const int checkEvenModulo = 2;
      if ((params.m_gaussianKernelSize % checkEvenModulo) != 1) {
        throw vpException(vpException::badValue, "Gaussian Kernel size should be odd.");
      }

      params.m_gaussianStdev = j.value("gaussianStdev", params.m_gaussianStdev);
      if (params.m_gaussianStdev <= 0) {
        throw vpException(vpException::badValue, "Standard deviation should be > 0");
      }

      params.m_gradientFilterKernelSize = j.value("gradientFilterKernelSize", params.m_gradientFilterKernelSize);
      if ((params.m_gradientFilterKernelSize % checkEvenModulo) != 1) {
        throw vpException(vpException::badValue, "Gradient filter kernel (Sobel or Scharr) size should be odd.");
      }

      std::string cannyBackendName = vpImageFilter::vpCannyBackendTypeToString(params.m_cannyBackendType);
      cannyBackendName = j.value("cannyBackendType", cannyBackendName);
      params.m_cannyBackendType = vpImageFilter::vpCannyBackendTypeFromString(cannyBackendName);
      params.m_lowerCannyThresh = j.value("lowerCannyThresh", params.m_lowerCannyThresh);
      params.m_lowerCannyThreshRatio = j.value("lowerThresholdRatio", params.m_lowerCannyThreshRatio);
      params.m_upperCannyThresh = j.value("upperCannyThresh", params.m_upperCannyThresh);
      params.m_upperCannyThreshRatio = j.value("upperThresholdRatio", params.m_upperCannyThreshRatio);
      params.m_edgeMapFilteringNbIter = j.value("edgeMapFilteringNbIter", params.m_edgeMapFilteringNbIter);

      params.m_centerXlimits = j.value("centerXlimits", params.m_centerXlimits);
      params.m_centerYlimits = j.value("centerYlimits", params.m_centerYlimits);
      std::pair<float, float> radiusLimits = j.value("radiusLimits", std::pair<float, float>(params.m_minRadius, params.m_maxRadius));
      params.m_minRadius = std::min<float>(radiusLimits.first, radiusLimits.second);
      params.m_maxRadius = std::max<float>(radiusLimits.first, radiusLimits.second);

      params.m_dilatationKernelSize = j.value("dilatationKernelSize", params.m_dilatationKernelSize);
      params.m_averagingWindowSize = j.value("averagingWindowSize", params.m_averagingWindowSize);
      if ((params.m_averagingWindowSize <= 0) || ((params.m_averagingWindowSize % checkEvenModulo) == 0)) {
        throw vpException(vpException::badValue, "Averaging window size must be positive and odd.");
      }

      params.m_centerMinThresh = j.value("centerThresh", params.m_centerMinThresh);
      if (params.m_centerMinThresh <= 0) {
        throw vpException(vpException::badValue, "Votes thresholds for center detection must be positive.");
      }

      params.m_expectedNbCenters = j.value("expectedNbCenters", params.m_expectedNbCenters);


      params.m_circleProbaThresh = j.value("circleProbabilityThreshold", params.m_circleProbaThresh);
      params.m_circleVisibilityRatioThresh = j.value("circleVisibilityRatioThreshold", params.m_circleVisibilityRatioThresh);

      params.m_circlePerfectness = j.value("circlePerfectnessThreshold", params.m_circlePerfectness);

      if ((params.m_circlePerfectness <= 0) || (params.m_circlePerfectness > 1)) {
        throw vpException(vpException::badValue, "Circle perfectness must be in the interval ] 0; 1].");
      }

      params.m_recordVotingPoints = j.value("recordVotingPoints", params.m_recordVotingPoints);

      params.m_centerMinDist = j.value("centerMinDistance", params.m_centerMinDist);
      if (params.m_centerMinDist <= 0) {
        throw vpException(vpException::badValue, "Centers minimum distance threshold must be positive.");
      }

      params.m_mergingRadiusDiffThresh = j.value("mergingRadiusDiffThresh", params.m_mergingRadiusDiffThresh);
      if (params.m_mergingRadiusDiffThresh <= 0) {
        throw vpException(vpException::badValue, "Radius difference merging threshold must be positive.");
      }
    }

    /**
     * \brief Parse a vpCircleHoughTransform into JSON format.
     *
     * \param[out] j : A JSON parser object.
     * \param[in] params : The circle Hough transform parameters that will be serialized in the json object.
     */
    friend inline void to_json(nlohmann::json &j, const vpCircleHoughTransformParams &params)
    {
      std::pair<float, float> radiusLimits = { params.m_minRadius, params.m_maxRadius };

      j = nlohmann::json {
          {"filteringAndGradientType", vpImageFilter::vpCannyFiltAndGradTypeToStr(params.m_filteringAndGradientType)},
          {"gaussianKernelSize", params.m_gaussianKernelSize},
          {"gaussianStdev", params.m_gaussianStdev},
          {"gradientFilterKernelSize", params.m_gradientFilterKernelSize},
          {"cannyBackendType", vpImageFilter::vpCannyBackendTypeToString(params.m_cannyBackendType)},
          {"lowerCannyThresh", params.m_lowerCannyThresh},
          {"lowerThresholdRatio", params.m_lowerCannyThreshRatio},
          {"upperCannyThresh", params.m_upperCannyThresh},
          {"upperThresholdRatio", params.m_upperCannyThreshRatio},
          {"edgeMapFilteringNbIter", params.m_edgeMapFilteringNbIter},
          {"centerXlimits", params.m_centerXlimits},
          {"centerYlimits", params.m_centerYlimits},
          {"radiusLimits", radiusLimits},
          {"dilatationKernelSize", params.m_dilatationKernelSize},
          {"averagingWindowSize", params.m_averagingWindowSize},
          {"centerThresh", params.m_centerMinThresh},
          {"expectedNbCenters", params.m_expectedNbCenters},
          {"circleProbabilityThreshold", params.m_circleProbaThresh},
          {"circleVisibilityRatioThreshold", params.m_circleVisibilityRatioThresh},
          {"circlePerfectnessThreshold", params.m_circlePerfectness},
          {"recordVotingPoints", params.m_recordVotingPoints},
          {"centerMinDistance", params.m_centerMinDist},
          {"mergingRadiusDiffThresh", params.m_mergingRadiusDiffThresh} };
    }
#endif

  private:
    // // Filtering + gradient operators to use
    vpImageFilter::vpCannyFilteringAndGradientType m_filteringAndGradientType; /*!< Permits to choose the filtering +
                                                                                    gradient operators to use.*/

    // // Gaussian smoothing attributes
    int m_gaussianKernelSize; /*!< Size of the Gaussian filter kernel used to smooth the input image.
                                   Must be an odd number.*/
    float m_gaussianStdev;   /*!< Standard deviation of the Gaussian filter.*/

    // // Gradient computation attributes
    int m_gradientFilterKernelSize; /*!< Size of the Sobel or Scharr kernels used to compute the gradients. Must be an odd number.*/

    // // Edge detection attributes
    float m_lowerCannyThresh; /*!< The lower threshold for the Canny operator. Values lower than this value are rejected.
                               A negative value makes the algorithm compute the lower threshold automatically.*/
    float m_upperCannyThresh; /*!< The upper threshold for the Canny operator. Only values greater than this value are marked as an edge.
                               A negative value makes the algorithm compute the upper and lower thresholds automatically.*/
    int m_edgeMapFilteringNbIter; /*!< Number of iterations of 8-neighbor connectivity filtering to apply to the edge map*/
    vpImageFilter::vpCannyBackendType m_cannyBackendType; /*!< Permits to choose the backend used to compute the edge map.*/
    float m_lowerCannyThreshRatio; /*!< The ratio of the upper threshold the lower threshold must be equal to.
                                        It is used only if the user asks to compute the Canny thresholds.*/
    float m_upperCannyThreshRatio; /*!< The ratio of pixels whose absolute gradient Gabs is lower or equal to define
                                        the upper threshold. It is used only if the user asks to compute the Canny thresholds.*/

    // // Center candidates computation attributes
    std::pair<int, int> m_centerXlimits; /*!< Minimum and maximum position on the horizontal axis of the center of the circle we want to detect.*/
    std::pair<int, int> m_centerYlimits; /*!< Minimum and maximum position on the vertical axis of the center of the circle we want to detect.*/
    float m_minRadius; /*!< Minimum radius of the circles we want to detect.*/
    float m_maxRadius; /*!< Maximum radius of the circles we want to detect.*/
    int m_dilatationKernelSize; /*!< Kernel size of the dilatation that is performed to detect the maximum number of votes for the center candidates.*/
    int m_averagingWindowSize; /*!< Size of the averaging window around the maximum number of votes to compute the
                                    center candidate such as it is the barycenter of the window. Must be odd.*/
    float m_centerMinThresh;  /*!< Minimum number of votes a point must exceed to be considered as center candidate.*/
    int m_expectedNbCenters; /*!< Expected number of different centers in the image. If negative, all candidates centers
                                  are kept, otherwise only up to this number are kept.*/

    // // Circle candidates computation attributes
    float m_circleProbaThresh;  /*!< Probability threshold in order to keep a circle candidate.*/
    float m_circlePerfectness; /*!< The threshold for the colinearity between the gradient of a point
                                    and the radius it would form with a center candidate to be able to vote.
                                    The formula to get the equivalent angle is: `angle = acos(circle_perfectness)`. */
    float m_circleVisibilityRatioThresh; /*!< Visibility ratio threshold: minimum ratio of the circle must be visible in order to keep a circle candidate.*/
    bool m_recordVotingPoints; /*!< If true, the edge-map points having voted for each circle will be stored.*/

    // // Circle candidates merging attributes
    float m_centerMinDist; /*!< Maximum distance between two circle candidates centers to consider merging them.*/
    float m_mergingRadiusDiffThresh; /*!< Maximum radius difference between two circle candidates to consider merging them.*/

    friend class vpCircleHoughTransform;
  };

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  typedef vpCircleHoughTransformParams vpCircleHoughTransformParameters;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /**
   * \brief Data storage for the computation of the center candidates.
   */
  typedef struct vpCenterVotes
  {
    std::pair<float, float> m_position;
    float m_votes;
  } vpCenterVotes;
#endif

/**
 * \brief Construct a new vpCircleHoughTransform object with default parameters.
 */
  vpCircleHoughTransform();

  /**
   * \brief Construct a new vpCircleHoughTransform object
   * from a \b vpCircleHoughTransformParams object.
   * \param[in] algoParams The parameters of the Circle Hough Transform.
   */
  VP_EXPLICIT vpCircleHoughTransform(const vpCircleHoughTransformParams &algoParams);

  /**
   * \brief Destroy the vp Circle Hough Transform object
   */
  virtual ~vpCircleHoughTransform();

  /** @name  Detection methods */
  //@{
#ifdef HAVE_OPENCV_CORE
  /**
   * \brief Perform Circle Hough Transform to detect the circles in an OpenCV image.
   *
   * \param[in] I The input gray scale image.
   * \return std::vector<vpImageCircle> The list of 2D circles detected in the image.
   */
  virtual std::vector<vpImageCircle> detect(const cv::Mat &cv_I);
#endif

  /**
   * \brief Convert the input image in a gray-scale image and then
   * perform Circle Hough Transform to detect the circles in it
   *
   * \param[in] I The input color image.
   * \return std::vector<vpImageCircle> The list of 2D circles detected in the image.
   */
  virtual std::vector<vpImageCircle> detect(const vpImage<vpRGBa> &I);

  /**
   * \brief Perform Circle Hough Transform to detect the circles in a gray-scale image
   *
   * \param[in] I The input gray scale image.
   * \return std::vector<vpImageCircle> The list of 2D circles detected in the image.
   */
  virtual std::vector<vpImageCircle> detect(const vpImage<unsigned char> &I);

  /**
   * \brief Perform Circle Hough Transform to detect the circles in in a gray-scale image.
   * Get only the \b nbCircles circles having the greatest number of votes.
   *
   * \param[in] I The input gray scale image.
   * \param[in] nbCircles The number of circles we want to get. If negative, all the circles will be
   * returned, sorted such as result[0] has the highest number of votes and result[end -1] the lowest.
   * \return std::vector<vpImageCircle> The list of 2D circles with the most number
   * of votes detected in the image.
   */
  virtual std::vector<vpImageCircle> detect(const vpImage<unsigned char> &I, const int &nbCircles);

  /*!
   * \brief Compute the mask containing pixels that voted for the \b detections.
   * \param[in] I The image for which we want to have the information.
   * \param[in] detections Vector containing the list of vpImageCircle for which we want to know the voting points.
   * \param[out] mask Optional mask where pixels to exclude have a value set to false.
   * \param[out] opt_votingPoints Optional vector of pairs of pixel coordinates that voted for the \b detections.
   */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  void computeVotingMask(const vpImage<unsigned char> &I, const std::vector<vpImageCircle> &detections,
                         std::optional< vpImage<bool> > &mask, std::optional<std::vector<std::vector<std::pair<unsigned int, unsigned int>>>> &opt_votingPoints) const;
#else
  void computeVotingMask(const vpImage<unsigned char> &I, const std::vector<vpImageCircle> &detections,
                         vpImage<bool> **mask, std::vector<std::vector<std::pair<unsigned int, unsigned int> > > **opt_votingPoints) const;
#endif

/** @name  Configuration from files */
//@{
#ifdef VISP_HAVE_NLOHMANN_JSON
  /**
   * \brief Construct a new vpCircleHoughTransform object configured according to
   * the JSON file whose path is \b jsonPath. Throw a \b vpException error if the file
   * does not exist.
   * \param[in] jsonPath The path towards the JSON configuration file.
   */
  VP_EXPLICIT vpCircleHoughTransform(const std::string &jsonPath);

  /**
   * \brief Initialize all the algorithm parameters using the JSON file
   * whose path is \b jsonPath. Throw a \b vpException error if the file
   * does not exist.
   *
   * \param[in] jsonPath The path towards the JSON configuration file.
   */
  virtual void initFromJSON(const std::string &jsonPath);

  /**
   * \brief Save the configuration of the detector in a JSON file
   * described by the path \b jsonPath. Throw a \b vpException
   * is the file cannot be created.
   *
   * \param[in] jsonPath The path towards the JSON output file.
   */
  virtual void saveConfigurationInJSON(const std::string &jsonPath) const;

  /**
   * \brief Read the detector configuration from JSON. All values are optional and if an argument is not present,
   * the default value defined in the constructor is kept
   *
   * \param[in] j The JSON object, resulting from the parsing of a JSON file.
   * \param[out] detector The detector, that will be initialized from the JSON data.
   */
  friend inline void from_json(const nlohmann::json &j, vpCircleHoughTransform &detector)
  {
    detector.m_algoParams = j;
  }

  /**
   * \brief Parse a vpCircleHoughTransform into JSON format.
   *
   * \param[out] j A JSON parser object.
   * \param[in] detector The vpCircleHoughTransform that must be parsed into JSON format.
   */
  friend inline void to_json(nlohmann::json &j, const vpCircleHoughTransform &detector)
  {
    j = detector.m_algoParams;
  }
#endif
  //@}

  /** @name  Setters */
  //@{
  /**
   * \brief Initialize all the algorithm parameters.
   *
   * \param[in] algoParams The algorithm parameters.
   */
  void init(const vpCircleHoughTransformParams &algoParams);

  /**
   * \brief Permits to choose the filtering + gradient operators to use.
   *
   * \param[in] type The type of filtering + gradient operators to use.
   */
  inline void setFilteringAndGradientType(const vpImageFilter::vpCannyFilteringAndGradientType &type)
  {
    m_algoParams.m_filteringAndGradientType = type;
    m_cannyVisp.setFilteringAndGradientType(type);
    initGradientFilters();
  }

  /**
   * \brief Set the parameters of the Gaussian filter, that permits to blur the
   * gradients of the image.
   *
   * \param[in] kernelSize The size of the Gaussian kernel. Must be an odd value.
   * \param[in] stdev The standard deviation of the Gaussian function.
   */
  inline void setGaussianParameters(const int &kernelSize, const float &stdev)
  {
    m_algoParams.m_gaussianKernelSize = kernelSize;
    m_algoParams.m_gaussianStdev = stdev;

    const unsigned int checkEvenModulo = 2;
    if ((m_algoParams.m_gaussianKernelSize % checkEvenModulo) != 1) {
      throw vpException(vpException::badValue, "Gaussian Kernel size should be odd.");
    }

    if (m_algoParams.m_gaussianStdev <= 0) {
      throw vpException(vpException::badValue, "Standard deviation should be > 0");
    }

    initGaussianFilters();
  }

  /**
   * \brief Set the parameters of the gradient filter (Sobel or Scharr) kernel size filters.
   *
   * \param[in] apertureSize The size of the gradient filters kernel. Must be an odd value.
   */
  inline void setGradientFilterAperture(const unsigned int &apertureSize)
  {
    m_algoParams.m_gradientFilterKernelSize = apertureSize;

    const unsigned int checkEvenModulo = 2;
    if ((m_algoParams.m_gradientFilterKernelSize % checkEvenModulo) != 1) {
      throw vpException(vpException::badValue, "Gradient filter (Sobel or Scharr) Kernel size should be odd.");
    }

    initGradientFilters();
  }

  /**
   * \brief Set the backend to use to perform the Canny edge detection.
   *
   * \param[in] type The backend that must be used.
   */
  inline void setCannyBackend(const vpImageFilter::vpCannyBackendType &type)
  {
    m_algoParams.m_cannyBackendType = type;
  }

  /*!
   * Set the threshold for the Canny operator.
   * Only value greater than this value are marked as an edge.
   * If negative, the threshold is automatically computed.
   * \param[in] lowerCannyThreshold : Canny filter lower threshold. When set to -1 (default), compute
   * automatically this threshold.
   * \param[in] upperCannyThreshold : Canny filter upper threshold. When set to -1 (default), compute
   * automatically this threshold.
   */
  inline void setCannyThreshold(const float &lowerCannyThreshold, const float &upperCannyThreshold)
  {
    m_algoParams.m_lowerCannyThresh = lowerCannyThreshold;
    m_algoParams.m_upperCannyThresh = upperCannyThreshold;
  }

  /**
   * \brief Set the Canny thresholds ratio that are used to automatically compute the Canny thresholds
   * in case the user asks to.
   *
   * \sa \ref vpCircleHoughTransform::setCannyThreshold "vpCircleHoughTransform::setCannyThreshold(const float&, const float&)"
   *
   * \param[in] lowerThreshRatio The ratio of the upper threshold the lower threshold will be equal to.
   * \param[in] upperThreshRatio The ratio of pixels that must have a gradient lower than the upper threshold.
   */
  inline void setCannyThresholdRatio(const float &lowerThreshRatio, const float &upperThreshRatio)
  {
    m_algoParams.m_lowerCannyThreshRatio = lowerThreshRatio;
    m_algoParams.m_upperCannyThreshRatio = upperThreshRatio;
    m_cannyVisp.setCannyThresholdsRatio(lowerThreshRatio, upperThreshRatio);
  }

  /*!
   * Set circles center min distance.
   * Change this value to detect circles with different distances to each other.
   *
   * \param[in] center_min_dist : Center min distance in pixels.
   */
  inline void setCircleCenterMinDist(const float &center_min_dist)
  {
    m_algoParams.m_centerMinDist = center_min_dist;

    if (m_algoParams.m_centerMinDist <= 0) {
      throw vpException(vpException::badValue, "Circles center min distance  must be positive.");
    }
  }

  /*!
   * Set circles center min and max location in the image.
   * If one value is equal to \b std::numeric_limits<int>::min or
   * \b std::numeric_limits<int>::max(), the algorithm will set it
   * either to -maxRadius or +maxRadius depending on if
   * it is the lower or upper limit that is missing.
   *
   * \param[in] center_min_x : Center min location on the horizontal axis, expressed in pixels.
   * \param[in] center_max_x : Center max location on the horizontal axis, expressed in pixels.
   * \param[in] center_min_y : Center min location on the vertical axis, expressed in pixels.
   * \param[in] center_max_y : Center max location on the vertical axis, expressed in pixels.
   */
  void setCircleCenterBoundingBox(const int &center_min_x, const int &center_max_x,
                                  const int &center_min_y, const int &center_max_y)
  {
    m_algoParams.m_centerXlimits.first = center_min_x;
    m_algoParams.m_centerXlimits.second = center_max_x;
    m_algoParams.m_centerYlimits.first = center_min_y;
    m_algoParams.m_centerYlimits.second = center_max_y;
  }

  /*!
   * Set circles min radius.
   * \param[in] circle_min_radius : Min radius in pixels.
   */
  inline void setCircleMinRadius(const float &circle_min_radius)
  {
    m_algoParams.m_minRadius = circle_min_radius;
  }

  /*!
   * Set circles max radius.
   * \param[in] circle_max_radius : Max radius in pixels.
   */
  inline void setCircleMaxRadius(const float &circle_max_radius)
  {
    m_algoParams.m_maxRadius = circle_max_radius;
  }

  /*!
   * \brief Set circles perfectness, which corresponds to the threshold of the colinearity
   * between the gradient of a point and the radius it would form with a center candidate
   * to be able to vote.
   * The formula to get the equivalent angle is: `angle = acos(circle_perfectness)`.
   * \param[in] circle_perfectness : Circle perfectness. Value between 0 and 1. A perfect circle has value 1.
   */
  void setCirclePerfectness(const float &circle_perfectness)
  {
    m_algoParams.m_circlePerfectness = circle_perfectness;
    if ((m_algoParams.m_circlePerfectness <= 0) || (m_algoParams.m_circlePerfectness > 1)) {
      throw vpException(vpException::badValue, "Circle perfectness must be in the interval ] 0; 1].");
    }
  }

  /**
   * \brief Set the parameters of the computation of the circle center candidates.
   *
   * \param[in] dilatationSize Kernel size of the dilatation operation used to detect the maxima in the center accumulator.
   * \param[in] centerThresh Minimum number of votes a point must exceed to be considered as center candidate.
   * \param[in] averagingWindowSize Size of the averaging window around the maximum number of votes to compute the
                                      center candidate such as it is the barycenter of the window. Must be odd.
   * \param[in] expectedNbCenters Expected number of centers in the image. If the number is negative, all the centers
   * are kept. Otherwise, maximum up to this number of centers are kept.
   */
  inline void setCenterComputationParameters(const int &dilatationSize, const float &centerThresh,
                                             const int &averagingWindowSize = 5, const int expectedNbCenters = -1)
  {
    m_algoParams.m_dilatationKernelSize = dilatationSize;
    m_algoParams.m_centerMinThresh = centerThresh;
    m_algoParams.m_averagingWindowSize = averagingWindowSize;
    m_algoParams.m_expectedNbCenters = expectedNbCenters;

    const int minDilatationKernel = 3;
    const unsigned int checkEvenModulo = 2;
    if (m_algoParams.m_dilatationKernelSize < minDilatationKernel) {
      throw vpException(vpException::badValue, "Dilatation kernel size for center detection must be greater or equal to 3.");
    }
    else if ((m_algoParams.m_dilatationKernelSize % checkEvenModulo) == 0) {
      throw vpException(vpException::badValue, "Dilatation kernel size for center detection must be odd.");
    }

    if (m_algoParams.m_centerMinThresh <= 0.f) {
      throw vpException(vpException::badValue, "Votes thresholds for center detection must be positive.");
    }

    if ((m_algoParams.m_averagingWindowSize <= 0) || ((m_algoParams.m_averagingWindowSize % checkEvenModulo) == 0)) {
      throw vpException(vpException::badValue, "Averaging window size must be positive and odd.");
    }
  }

  /**
   * \brief Set the parameters of the computation of the circle radius candidates.
   *
   * \param[in] radiusRatioThresh Minimum number of votes per radian a radius candidate RC_ij of a center candidate CeC_i must have in order that the circle of center CeC_i and radius RC_ij must be considered as circle candidate.
   */
  inline void setRadiusRatioThreshold(const float &radiusRatioThresh)
  {
    m_algoParams.m_circleProbaThresh = radiusRatioThresh;

    if (m_algoParams.m_circleProbaThresh <= 0) {
      throw vpException(vpException::badValue, "Radius ratio threshold must be > 0.");
    }
  }

  /**
   * \brief Set the radius merging threshold used during the merging step in order
   * to merge the circles that are similar.
   *
   * \param[in] radiusDifferenceThresh Maximum radius difference between two circle candidates to consider merging them.
   */
  inline void setRadiusMergingThresholds(const float &radiusDifferenceThresh)
  {
    m_algoParams.m_mergingRadiusDiffThresh = radiusDifferenceThresh;

    if (m_algoParams.m_mergingRadiusDiffThresh <= 0) {
      throw vpException(vpException::badValue, "Radius difference merging threshold must be positive.");
    }
  }

  inline void setMask(const vpImage<bool> &mask)
  {
    mp_mask = &mask;
  }
  //@}

  /** @name  Getters */
  //@{
  /**
   * \brief Get the list of Center Candidates, stored as pair <idRow, idCol>
   *
   * \return std::vector<std::pair<float, float> > The list of Center Candidates, stored as pair <idRow, idCol>
   */
  inline std::vector<std::pair<float, float> > getCenterCandidatesList() const
  {
    return m_centerCandidatesList;
  }

  /**
   * \brief Get the number of votes of each Center Candidates.
   *
   * \return std::vector<int> The number of votes of each Center Candidates, ordered in the same way than \b m_centerCandidatesList.
   */
  inline std::vector<int> getCenterCandidatesVotes() const
  {
    return m_centerVotes;
  }

  /**
   * \brief Get the Circle Candidates before merging step.
   *
   * \return std::vector<vpImageCircle> The list of circle candidates
   * that were obtained before the merging step.
   */
  inline std::vector<vpImageCircle> getCircleCandidates() const
  {
    return m_circleCandidates;
  }

  /**
   * \brief Get the probabilities of the Circle Candidates.
   *
   * \return std::vector<float> The votes accumulator.
   */
  inline std::vector<float> getCircleCandidatesProbabilities() const
  {
    return m_circleCandidatesProbabilities;
  }

  /**
   * \brief Get the votes of the circle candidates.
   *
   * \return std::vector<unsigned int> The votes of the circle candidates.
   */
  inline std::vector<unsigned int> getCircleCandidatesVotes() const
  {
    return m_circleCandidatesVotes;
  }

  /**
   * \brief Get the gradient along the horizontal axis of the image.
   *
   * \return vpImage<float> The gradient along the horizontal axis of  the image.
   */
  inline vpImage<float> getGradientX() const
  {
    return m_dIx;
  }

  /**
   * \brief Get the gradient along the vertical axis of the image.
   *
   * \return vpImage<float> The gradient along the vertical axis of  the image.
   */
  inline vpImage<float> getGradientY() const
  {
    return m_dIy;
  }

  /**
   * \brief Get the Edge Map computed thanks to the Canny edge filter.
   *
   * \return vpImage<unsigned char> The edge map computed during the edge detection step.
   */
  inline vpImage<unsigned char> getEdgeMap() const
  {
    return m_edgeMap;
  }

  /*!
   * Get internal Canny filter upper threshold. When value is equal to -1 (default), it means that the threshold is computed
   * automatically.
   */
  inline float getCannyThreshold() const
  {
    return m_algoParams.m_upperCannyThresh;
  }

  /*!
   * Get circles center min distance in pixels.
   */
  inline float getCircleCenterMinDist() const
  {
    return m_algoParams.m_centerMinDist;
  }

  /*!
   * Get circles min radius in pixels.
   */
  inline float getCircleMinRadius() const
  {
    return m_algoParams.m_minRadius;
  }

  /*!
   * Get circles max radius in pixels.
   */
  inline float getCircleMaxRadius() const
  {
    return m_algoParams.m_maxRadius;
  }

  /*!
   * Get the probabilities of the detections that are outputed by vpCircleHoughTransform::detect()
   */
  inline std::vector<float> getDetectionsProbabilities() const
  {
    return m_finalCirclesProbabilities;
  }

  /*!
   * Get the number of votes for the detections that are outputed by vpCircleHoughTransform::detect()
   */
  inline std::vector<unsigned int> getDetectionsVotes() const
  {
    return m_finalCircleVotes;
  }
  //@}

  /*!
   * Create a string with all Hough transform parameters.
   */
  std::string toString() const;

  /*!
   * Create a ostream with all Hough transform parameters.
   */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpCircleHoughTransform &detector);

  static const unsigned char edgeMapOn;
  static const unsigned char edgeMapOff;

protected:
  /**
   * \brief Initialize the Gaussian filters used to blur the image and
   * compute the gradient images.
   */
  virtual void initGaussianFilters();

  /**
   * \brief Initialize the gradient filters used to compute the gradient images.
   */
  virtual void initGradientFilters();

  /**
   * \brief Perform Gaussian smoothing on the input image to reduce the noise
   * that would perturbate the edge detection.
   * Then, compute the x-gradient and y-gradient of the input images.
   *
   * \param[in] I The input gray scale image.
   */
  virtual void computeGradients(const vpImage<unsigned char> &I);

  /**
   * \brief Perform edge detection based on the computed gradients.
   * Stores the edge points and the edge points connectivity.
   *
   * \param[in] I The input gray scale image.
   */
  virtual void edgeDetection(const vpImage<unsigned char> &I);

  /**
   * \brief Filter the edge map in order to remove isolated edge points.
   */
  virtual void filterEdgeMap();

  /**
   * \brief Determine the image points that are circle center candidates.
   * Increment the center accumulator based on the edge points and gradient information.
   * Perform thresholding to keep only the center candidates that exceed the threshold.
   */
  virtual void computeCenterCandidates();

  /**
   * \brief Aggregate center candidates that are close to each other.
   * \param[in] peak_positions_votes Vector containing raw center candidates.
   */
  virtual void filterCenterCandidates(const std::vector<vpCenterVotes> &peak_positions_votes);

  /**
   * \brief Compute the probability of \b circle given the number of pixels voting for
   * it \b nbVotes.
   * The probability is defined as the ratio of \b nbVotes by the theoretical number of
   * pixel that should be visible in the image.
   *
   * \param[in] circle The circle for which we want to evaluate the probability.
   * \param[in] nbVotes The number of visible pixels of the given circle.
   * \return float The probability of the circle.
   */
  virtual float computeCircleProbability(const vpImageCircle &circle, const unsigned int &nbVotes);

  /**
   * \brief For each center candidate CeC_i, do:
   * - For each edge point EP_j, compute the distance d_ij = distance(CeC_i; EP_j)
   * - Determine to which radius candidate bin RCB_k the distance d_ij belongs to
   * - Increment the radius candidate accumulator accum_rc[CeC_i][RCB_k]
   * - If accum_rc[CeC_i][RCB_k] > radius_count_thresh, add the circle candidate (CeC_i, RCB_k)
   *   to the list of circle candidates
   */
  virtual void computeCircleCandidates();

  /**
   * \brief For each circle candidate CiC_i, check if similar circles have also been detected and if so merges them.
   */
  virtual void mergeCircleCandidates();

  /**
   * \brief For each circle candidate CiC_i do:
   * - For each other circle candidate CiC_j do:
   * +- Compute the similarity between CiC_i and CiC_j
   * +- If the similarity exceeds a threshold, merge the circle candidates CiC_i and CiC_j and remove CiC_j of the list
   * - Add the circle candidate CiC_i to the final list of detected circles
   * \param[out] circleCandidates List of circle candidates in which we want to merge the similar circles.
   * \param[out] circleCandidatesVotes List of votes of the circle candidates.
   * \param[out] circleCandidatesProba List of probabilities of the circle candidates.
   * \param[out] votingPoints List of edge-map points having voted of the circle candidates.
   */
  virtual void mergeCandidates(std::vector<vpImageCircle> &circleCandidates, std::vector<unsigned int> &circleCandidatesVotes,
                               std::vector<float> &circleCandidatesProba, std::vector<std::vector<std::pair<unsigned int, unsigned int> > > &votingPoints);

  vpCircleHoughTransformParams m_algoParams; /*!< Attributes containing all the algorithm parameters.*/
  // // Gaussian smoothing attributes
  vpArray2D<float> m_fg;

  // // Gradient computation attributes
  const vpImage<bool> *mp_mask; /*!< Mask that permits to avoid to compute gradients on some regions of the image.*/
  vpArray2D<float> m_gradientFilterX; /*!< Contains the coefficients of the gradient kernel along the X-axis*/
  vpArray2D<float> m_gradientFilterY; /*!< Contains the coefficients of the gradient kernel along the Y-axis*/
  vpImage<float> m_dIx; /*!< Gradient along the x-axis of the input image.*/
  vpImage<float> m_dIy; /*!< Gradient along the y-axis of the input image.*/

  // // Edge detection attributes
  vpCannyEdgeDetection m_cannyVisp; /*!< Edge detector ViSP implementation, used if ViSP has not been compiled with OpenCV imgproc module*/
  vpImage<unsigned char> m_edgeMap; /*!< Edge map resulting from the edge detection algorithm.*/

  // // Center candidates computation attributes
  std::vector<std::pair<unsigned int, unsigned int> > m_edgePointsList; /*!< Vector that contains the list of edge points, to make faster some parts of the algo. They are stored as pair <row, col>.*/
  std::vector<std::pair<float, float> > m_centerCandidatesList; /*!< Vector that contains the list of center candidates. They are stored as pair <row, col>.*/
  std::vector<int> m_centerVotes; /*!< Number of votes for the center candidates that are kept.*/

  // // Circle candidates computation attributes
  std::vector<vpImageCircle> m_circleCandidates;        /*!< List of the candidate circles.*/
  std::vector<float> m_circleCandidatesProbabilities; /*!< Probabilities of each candidate circle that is kept.*/
  std::vector<unsigned int> m_circleCandidatesVotes; /*!< Number of pixels voting for each candidate circle that is kept.*/
  std::vector<std::vector<std::pair<unsigned int, unsigned int> > > m_circleCandidatesVotingPoints; /*!< Points that voted for each circle candidate.*/

  // // Circle candidates merging attributes
  std::vector<vpImageCircle> m_finalCircles; /*!< List of the final circles, i.e. the ones resulting from the merge of the circle candidates.*/
  std::vector<float> m_finalCirclesProbabilities; /*!< Probabilities of each final circle, i.e. resulting from the merge of the circle candidates.*/
  std::vector<unsigned int> m_finalCircleVotes; /*!< Number of votes for the final circles.*/
  std::vector<std::vector<std::pair<unsigned int, unsigned int> > > m_finalCirclesVotingPoints; /*!< Points that voted for each final circle.*/
};

END_VISP_NAMESPACE

#endif
