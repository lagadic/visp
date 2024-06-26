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

#ifndef VP_CANNY_EDGE_DETECTION_H
#define VP_CANNY_EDGE_DETECTION_H

// System includes
#include <map>
#include <vector>

// ViSP include
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>

// 3rd parties include
#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/**
 * \brief Class that implements the Canny's edge detector.
 * It is possible to use a boolean mask to ignore some pixels of
 * the input gray-scale image.
*/
class VISP_EXPORT vpCannyEdgeDetection
{

public:
  /** @name Constructors and initialization */
  //@{
  /**
   * \brief Default constructor of the vpCannyEdgeDetection class.
   * The thresholds used during the hysteresis thresholding step are set to be automatically computed.
   */
  vpCannyEdgeDetection();

  /**
   * \brief Construct a new vpCannyEdgeDetection object that uses Gaussian blur + Sobel operators to compute
   * the edge map.
   *
   * \param[in] gaussianKernelSize : The size of the Gaussian filter kernel. Must be odd.
   * \param[in] gaussianStdev : The standard deviation of the Gaussian filter.
   * \param[in] sobelAperture : The size of the Sobel filters kernel. Must be odd.
   * \param[in] lowerThreshold : The lower threshold of the hysteresis thresholding step. If negative, will be computed
   * from the upper threshold.
   * \param[in] upperThreshold : The upper threshold of the hysteresis thresholding step. If negative, will be computed
   * from the histogram of the absolute gradient.
   * \param[in] lowerThresholdRatio : If the thresholds must be computed,the lower threshold will be equal to the upper
   * threshold times \b lowerThresholdRatio .
   * \param[in] upperThresholdRatio : If the thresholds must be computed,the upper threshold will be equal to the value
   * such as the number of pixels of the image times \b upperThresholdRatio have an absolute gradient lower than the
   * upper threshold.
   * \param[in] filteringType : The filtering and gradient operators to apply to the image before the edge detection
   * operation.
   */
  vpCannyEdgeDetection(const int &gaussianKernelSize, const float &gaussianStdev, const unsigned int &sobelAperture,
                       const float &lowerThreshold = -1.f, const float &upperThreshold = -1.f,
                       const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                       const vpImageFilter::vpCannyFilteringAndGradientType &filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING);

  // // Configuration from files
#ifdef VISP_HAVE_NLOHMANN_JSON
  /**
   * \brief Construct a new vpCannyEdgeDetection object.
   *
   * \param[in] jsonPath : The path towards the JSON file to use to initialize the vpCannyEdgeDetection object.
   */
  vpCannyEdgeDetection(const std::string &jsonPath);

  /**
   * \brief Initialize all the algorithm parameters using the JSON file
   * whose path is \b jsonPath. Throw a \b vpException error if the file
   * does not exist.
   *
   * \param[in] jsonPath : The path towards the JSON configuration file.
   */
  void initFromJSON(const std::string &jsonPath);

  /**
   * \brief Read the detector configuration from JSON. All values are optional and if an argument is not present,
   * the default value defined in the constructor is kept
   *
   * \param[in] j : The JSON object, resulting from the parsing of a JSON file.
   * \param[out] detector : The detector that will be initialized from the JSON data.
   */
  friend void from_json(const nlohmann::json &j, vpCannyEdgeDetection &detector);

  /**
   * \brief Parse a vpCannyEdgeDetection object into JSON format.
   *
   * \param[out] j : A JSON parser object.
   * \param[in] detector : The vpCannyEdgeDetection object that must be parsed into JSON format.
   */
  friend void to_json(nlohmann::json &j, const vpCannyEdgeDetection &detector);
#endif
  //@}

  /** @name  Detection methods */
  //@{
#ifdef HAVE_OPENCV_CORE
  /**
   * \brief Detect the edges in an image.
   * Convert the color image into a ViSP gray-scale image.
   *
   * \param[in] cv_I A color image, in OpenCV format.
   * \return vpImage<unsigned char> 255 means an edge, 0 means not an edge.
   */
  vpImage<unsigned char> detect(const cv::Mat &cv_I);
#endif

  /**
   * \brief Detect the edges in an image.
   * Convert the color image into a gray-scale image.
   *
   * \param[in] I_color : An RGB image, in ViSP format.
   * \return vpImage<unsigned char> 255 means an edge, 0 means not an edge.
   */
  vpImage<unsigned char> detect(const vpImage<vpRGBa> &I_color);

  /**
   * \brief Detect the edges in a gray-scale image.
   *
   * \param[in] I : A gray-scale image, in ViSP format.
   * \return vpImage<unsigned char> 255 means an edge, 0 means not an edge.
   */
  vpImage<unsigned char> detect(const vpImage<unsigned char> &I);
  //@}

  /** @name  Setters */
  //@{
  /**
   * \brief Set the Filtering And Gradient operators to apply to the image before the edge detection operation.
   *
   * \param[in] type The operators to apply.
   */
  inline void setFilteringAndGradientType(const vpImageFilter::vpCannyFilteringAndGradientType &type)
  {
    m_filteringAndGradientType = type;
    initGradientFilters();
  }

  /**
   * \brief Set the Gradients of the image that will be processed.
   *
   * \param[in] dIx : Gradient along the horizontal axis of the image.
   * \param[in] dIy : Gradient along the vertical axis of the image.
   */
  inline void setGradients(const vpImage<float> &dIx, const vpImage<float> &dIy)
  {
    m_dIx = dIx;
    m_dIy = dIy;
    m_areGradientAvailable = true;
  }

  /**
   * \brief Set the lower and upper Canny Thresholds used to qualify the edge point candidates.
   * Edge point candidates whose gradient is between these two values is kept only if it
   * linked somehow to a strong edge point.
   *
   * \param[in] lowerThresh : The lower threshold: each point whose gradient is below this threshold is discarded.
   * When lower threshold value is negative, Canny recommendation is applied to have the lower threshold 3 times lower
   * than the upper threshold.
   * \param[in] upperThresh : The upper threshold: each point whose gradient is greater than this threshold is
   * said to be a strong edge point and is kept.
   */
  inline void setCannyThresholds(const float &lowerThresh, const float &upperThresh)
  {
    m_lowerThreshold = lowerThresh;
    m_upperThreshold = upperThresh;
  }

  /**
   * \brief Set the lower and upper Canny Thresholds ratio that are used to compute them automatically. To ask to
   * compute automatically the thresholds, you must set the lower and upper thresholds with negative values using the
   * appropriate setter.
   *
   * \sa \ref vpCannyEdgeDetection::setCannyThresholds() "vpCannyEdgeDetection::setCannyThresholds(const float&, const float&)"
   * \param[in] lowerThreshRatio : The lower threshold ratio: if the thresholds are computed automatically, the lower
   * threshold will be equal to the upper threshold multiplied by \b lowerThreshRatio.
   * \param[in] upperThreshRatio : The upper threshold ratio: if the thresholds are computed automatically, the  upper
   * threshold will be set such as \b upperThreshRatio times the number of pixels of the image have their absolute
   * gradient lower then the upper threshold.
   */
  inline void setCannyThresholdsRatio(const float &lowerThreshRatio, const float &upperThreshRatio)
  {
    m_lowerThresholdRatio = lowerThreshRatio;
    m_upperThresholdRatio = upperThreshRatio;
  }

  /**
   * \brief Set the Gaussian Filters kernel size and standard deviation
   * and initialize the aforementioned filters.
   *
   * \param[in] kernelSize : The size of the Gaussian filters kernel.
   * \param[in] stdev : The standard deviation of the Gaussian filters used to blur and
   * compute the gradient of the image.
   */
  inline void setGaussianFilterParameters(const int &kernelSize, const float &stdev)
  {
    m_gaussianKernelSize = kernelSize;
    m_gaussianStdev = stdev;
    initGaussianFilters();
  }

  /**
   * \brief  Set the parameters of the gradient filter (Sobel or Scharr) kernel size filters.
   *
   * \param[in] apertureSize The size of the gradient filters kernel. Must be an odd value.
   */
  inline void setGradientFilterAperture(const unsigned int &apertureSize)
  {
    m_gradientFilterKernelSize = apertureSize;
    initGradientFilters();
  }

  /**
   * \brief Set a mask to ignore pixels for which the mask is false.
   *
   * \warning The mask must be reset manually by the user (either for another mask
   * or set to \b nullptr ) before computing the edge-map of another image.
   *
   * @param p_mask If different of \b nullptr , a mask of booleans where \b true
   * indicates that a pixel must be considered and \b false that the pixel should
   * be ignored.
   */
  inline void setMask(const vpImage<bool> *p_mask)
  {
    mp_mask = p_mask;
  }
  //@}
private:
  typedef enum EdgeType
  {
    STRONG_EDGE, /*!< This pixel exceeds the upper threshold of the double hysteresis phase, it is thus for sure an edge point.*/
    WEAK_EDGE,/*!< This pixel is between the lower and upper threshold of the double hysteresis phase, it is an edge point only if it is linked at some point to an edge point.*/
    ON_CHECK /*!< This pixel is currently tested to know if it is linked to a strong edge point.*/
  } EdgeType;

  // Filtering + gradient methods choice
  vpImageFilter::vpCannyFilteringAndGradientType m_filteringAndGradientType; /*!< Choice of the filter and
      gradient operator to apply before the edge detection step*/

  // // Gaussian smoothing attributes
  int m_gaussianKernelSize; /*!< Size of the Gaussian filter kernel used to smooth the input image. Must be an odd number.*/
  float m_gaussianStdev;   /*!< Standard deviation of the Gaussian filter.*/
  vpArray2D<float> m_fg; /*!< Array that contains the Gaussian kernel.*/

  // // Gradient computation attributes
  bool m_areGradientAvailable; /*!< Set to true if the user provides the gradient images, false otherwise. In the latter case, the class will compute the gradients.*/
  unsigned int m_gradientFilterKernelSize; /*!< The size of the Sobel kernels used to compute the gradients of the image.*/
  vpArray2D<float> m_gradientFilterX; /*!< Array that contains the gradient filter kernel (Sobel or Scharr) along the X-axis.*/
  vpArray2D<float> m_gradientFilterY; /*!< Array that contains the gradient filter kernel (Sobel or Scharr) along the Y-axis.*/
  vpImage<float> m_dIx; /*!< X-axis gradient.*/
  vpImage<float> m_dIy; /*!< Y-axis gradient.*/

  // // Edge thining attributes
  std::map<std::pair<unsigned int, unsigned int>, float> m_edgeCandidateAndGradient; /*!< Map that contains point image coordinates and corresponding gradient value.*/

  // // Hysteresis thresholding attributes
  float m_lowerThreshold; /*!< Lower threshold for the hysteresis step. If negative, it will be deduced
                               as from m_upperThreshold. */
  float m_lowerThresholdRatio; /*!< If the thresholds must be computed, the ratio of the upper threshold the lower
                                    threshold is equal: m_lowerThreshold = m_lowerThresholdRatio * m_upperThreshold. */
  float m_upperThreshold; /*!< Upper threshold for the hysteresis step.*/
  float m_upperThresholdRatio; /*!< If the thresholds must be computed, the ratio of pixels of the gradient image that
                                    must be lower than the upper threshold \b m_upperThreshold.*/

  // // Edge tracking attributes
  std::map<std::pair<unsigned int, unsigned int>, EdgeType> m_edgePointsCandidates; /*!< Map that contains the strong edge points, i.e. the points for which we know for sure they are edge points,
                                                and the weak edge points, i.e. the points for which we still must determine if they are actual edge points.*/
  vpImage<unsigned char> m_edgeMap; /*!< Final edge map that results from the whole Canny algorithm.*/
  const vpImage<bool> *mp_mask; /*!< Mask that permits to consider only the pixels for which the mask is true.*/

  /** @name Constructors and initialization */
  //@{
  /**
   * \brief Initialize the Gaussian filters used to filter the input image.
   */
  void initGaussianFilters();

  /**
   * \brief Initialize the gradient filters (Sobel or Scharr) used to compute the input image gradients.
   */
  void initGradientFilters();
  //@}

  /** @name Different steps methods */
  /**
   * \brief Step 1: filtering + Step 2: gradient computation
   * \details First, perform Gaussian blur to the input image.
   * Then, compute the x-axis and y-axis gradients of the image.
   * \param[in] I : The image we want to compute the gradients.
   */
  void computeFilteringAndGradient(const vpImage<unsigned char> &I);

  /**
   * \brief Step 3: Edge thining.
   * \details Perform the edge thining step.
   * Perform a non-maximum suppression to keep only local maxima as edge candidates.
   * \param[in] lowerThreshold Edge candidates that are below this threshold are definitely not
   * edges.
   */
  void performEdgeThinning(const float &lowerThreshold);

  /**
   * \brief Perform hysteresis thresholding.
   * \details Edge candidates that are greater than \b m_upperThreshold are saved in \b m_strongEdgePoints
   * and will be kept in the final edge map.
   * Edge candidates that are between \b m_lowerThreshold and \b m_upperThreshold are saved in
   * \b m_weakEdgePoints and will be kept in the final edge map only if they are connected
   * to a strong edge point.
   * Edge candidates that are below \b m_lowerThreshold are discarded.
   * \param[in] lowerThreshold Edge candidates that are below this threshold are definitely not
   * edges.
   * \param[in] upperThreshold Edge candidates that are greater than this threshold are classified
   * as strong edges.
   */
  void performHysteresisThresholding(const float &lowerThreshold, const float &upperThreshold);

  /**
   * \brief Search recursively for a strong edge in the neighborhood of a weak edge.
   *
   * \param[in] coordinates : The coordinates we are checking.
   * \return true We found a strong edge point in its 8-connected neighborhood.
   * \return false We did not found a strong edge point in its 8-connected neighborhood.
   */
  bool recursiveSearchForStrongEdge(const std::pair<unsigned int, unsigned int> &coordinates);

  /**
   * \brief Perform edge tracking.
   * \details For each weak edge, we will recursively check if they are 8-connected to a strong edge point.
   * If so, the weak edge will be saved in \b m_strongEdgePoints and will be kept in the final edge map.
   * Otherwise, the edge point will be discarded.
   */
  void performEdgeTracking();
  //@}
};
END_VISP_NAMESPACE
#endif
