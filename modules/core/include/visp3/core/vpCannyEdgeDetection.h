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
#include <iostream>
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <sys/resource.h> // To dynamically change the stack size
#endif

// ViSP include
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpRGBa.h>

// 3rd parties include
#ifdef VISP_HAVE_NLOHMANN_JSON
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE
/**
 * \brief Class that implements the Canny's edge detector.
 * It is possible to use a boolean mask to ignore some pixels of
 * the input gray-scale image.
 *
 * \warning If the stack size is not sufficient, a SEGFAULT can occur on some images which
 * are really over-exposed.
 * \note The maximum stack on MacOS seems to be 65532000 bytes, see https://stackoverflow.com/a/13261334
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
   * \param[in] storeEdgePoints : If true, the list of edge-points will be available using
   * \param[in] nbThread : Number of thread to use. -1 to let the program choose.
   * \b vpCannyEdgeDetection::getEdgePointsList().
   */
  vpCannyEdgeDetection(const int &gaussianKernelSize, const float &gaussianStdev, const unsigned int &sobelAperture,
                       const float &lowerThreshold = -1.f, const float &upperThreshold = -1.f,
                       const float &lowerThresholdRatio = 0.6f, const float &upperThresholdRatio = 0.8f,
                       const vpImageFilter::vpCannyFilteringAndGradientType &filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING,
                       const bool &storeEdgePoints = false, const int &nbThread = -1);

  /**
   * \brief Reinitialize the detector:
   * - compute the number of threads to use if applicable
   * - initialize the Gaussian filters
   * - initialize the derivative filters
   * - forget the mask (but do not alter the memory)
   * - forget GIx and GIy (but do not alter the memory)
   * - delete previous results
   *
   * \return * void
   */
  void reinit();

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
   * \warning If the stack size is not sufficient, a SEGFAULT can occur on some images which
   * are really over-exposed.
   *
   * \param[in] cv_I A color image, in OpenCV format.
   * \return vpImage<unsigned char> 255 means an edge, 0 means not an edge.
   */
  vpImage<unsigned char> detect(const cv::Mat &cv_I);
#endif

  /**
   * \brief Detect the edges in an image.
   * Convert the color image into a gray-scale image.
   * \warning If the stack size is not sufficient, a SEGFAULT can occur on some images which
   * are really over-exposed.
   *
   * \param[in] I_color : An RGB image, in ViSP format.
   * \return vpImage<unsigned char> 255 means an edge, 0 means not an edge.
   */
  vpImage<unsigned char> detect(const vpImage<vpRGBa> &I_color);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename ArithmeticType, bool useFullScale>
  vpImage<unsigned char> detect(const vpImage<vpHSV<ArithmeticType, useFullScale>> &Ihsv)
  {
    // // Step 1 and 2: filter the image and compute the gradient, if not given by the user
    if (!m_areGradientAvailable) {
      vpImage<vpHSV<ArithmeticType, useFullScale>> Iblur;
      vpImageFilter::gaussianBlur(Ihsv, Iblur, m_gaussianKernelSize, m_gaussianStdev, true, mp_mask);
      vpImageFilter::gradientFilter(Iblur, m_dIx, m_dIy, m_nbThread, mp_mask, m_filteringAndGradientType);
    }
    m_areGradientAvailable = false; // Reset for next call

    // // Step 3: edge thining
    float upperThreshold = m_upperThreshold;
    float lowerThreshold = m_lowerThreshold;
    if (upperThreshold < 0) {
      upperThreshold = vpImageFilter::computeCannyThreshold(Ihsv, lowerThreshold, &m_dIx, &m_dIy, m_gaussianKernelSize,
                                                            m_gaussianStdev, m_lowerThresholdRatio,
                                                            m_upperThresholdRatio, m_filteringAndGradientType, mp_mask);
    }
    else if (m_lowerThreshold < 0) {
      // Applying Canny recommendation to have the upper threshold 3 times greater than the lower threshold.
      lowerThreshold = m_upperThreshold / 3.f;
    }
    // To ensure that if lowerThreshold = 0, we reject null gradient points
    lowerThreshold = std::max<float>(lowerThreshold, std::numeric_limits<float>::epsilon());

    step3to5(Ihsv.getHeight(), Ihsv.getWidth(), lowerThreshold, upperThreshold);
    return m_edgeMap;
  }
#endif

  /**
   * \brief Detect the edges in a gray-scale image.
   * \warning If the stack size is not sufficient, a SEGFAULT can occur on some images which
   * are really over-exposed.
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
   * \param p_mask If different of \b nullptr , a mask of booleans where \b true
   * indicates that a pixel must be considered and \b false that the pixel should
   * be ignored.
   */
  inline void setMask(const vpImage<bool> *p_mask)
  {
    mp_mask = p_mask;
  }

  /**
   * \brief Set the minimum stack size, expressed in bytes, due to the recursive algorithm.
   * If not called, the stack size is left at its default value when running the
   * Canny edge detection algorithm.
   *
   * \note The stack size is changed back to its original value after
   * before leaving the detect() function.
   * \note On Windows, the minimum stack size is defined at compilation time
   * and cannot be changed during runtime.
   * \note The maximum stack on MacOS seems to be 65532000 bytes, see https://stackoverflow.com/a/13261334
   * \warning If the stack size is not sufficient, a SEGFAULT can occur on some images which
   * are really over-exposed.
   *
   * \param[in] requiredStackSize The required stack size, in bytes.
   */
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  inline void setMinimumStackSize(const rlim_t &requiredStackSize)
  {
    m_minStackSize = requiredStackSize;
  }
#else
  inline void setMinimumStackSize(const unsigned int &requiredStackSize)
  {
    (void)requiredStackSize;
    static bool hasNotBeenDisplayed = true;
    if (hasNotBeenDisplayed) {
      std::cerr << "vpCannyEdgeDetection::setStackSize() has no effect on non-POSIX systems. The stack size is defined during compilation." << std::endl;
      hasNotBeenDisplayed = false;
    }
  }
#endif

  inline void setNbThread(const int &maxNbThread)
  {
#ifdef VISP_HAVE_OPENMP
    int nbThread = maxNbThread;
    if (nbThread < 0) {
      nbThread = omp_get_max_threads();
    }
    m_nbThread = nbThread;
#else
    m_nbThread = 1;
    std::cout << "[WARNING] OpenMP is not available, setting the number of threads is ignored." << std::endl;
#endif
  }
  /**
   * \brief If set to true, the list of the detected edge-points will be available
   * calling the method \b vpCannyEdgeDetection::getEdgePointsList().
   *
   * \param[in] storeEdgePoints The new desired status.
   */
  inline void setStoreEdgePoints(const bool &storeEdgePoints)
  {
    m_storeListEdgePoints = storeEdgePoints;
  }
  //@}

  /** @name  Getters */
  //@{
  /**
   * \brief Get the list of edge-points that have been detected.
   *
   * \return std::vector<vpImagePoint> The edge-points list.
   */
  inline std::vector<vpImagePoint> getEdgePointsList() const
  {
    if (!m_storeListEdgePoints) {
      throw(vpException(vpException::fatalError, "Asking for the edge-points list while not asking to store it"));
    }
    return m_edgePointsList;
  }

  /**
   * \brief Get the minimum stack size used by the algorithm.
   *
   * \note On Windows, the minimum stack size is defined at compilation time
   * and cannot be changed during runtime.
   *
   * \return rlim_t The minimum stack size.
   */
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  inline rlim_t getMinimumStackSize() const
  {
    return m_minStackSize;
  }
#else
  inline unsigned int getMinimumStackSize() const
  {
    const unsigned int limit = 65532000;
    return limit;
  }
#endif

  /**
   * \brief Get the horizontal gradient.
   *
   * \return const vpImage<float>& GIx
   */
  const vpImage<float> &getGIx() const
  {
    return m_dIx;
  }

  /**
   * \brief Get the vertical gradient.
   *
   * \return const vpImage<float>& GIy
   */
  const vpImage<float> &getGIy() const
  {
    return m_dIy;
  }

  /**
   * \brief Get the final edge-map.
   *
   * \return const vpImage<float>& The edge-map
   */
  const vpImage<unsigned char> &getEdgeMap() const
  {
    return m_edgeMap;
  }

  //@}
private:
  typedef enum EdgeType
  {
    STRONG_EDGE, /*!< This pixel exceeds the upper threshold of the double hysteresis phase, it is thus for sure an edge point.*/
    WEAK_EDGE,/*!< This pixel is between the lower and upper threshold of the double hysteresis phase, it is an edge point only if it is linked at some point to an edge point.*/
    ON_CHECK, /*!< This pixel is currently tested to know if it is linked to a strong edge point.*/
    NOT_EDGE /*!< This pixel is surely not an edge.*/
  } EdgeType;

  // Filtering + gradient methods choice
  vpImageFilter::vpCannyFilteringAndGradientType m_filteringAndGradientType; /*!< Choice of the filter and
      gradient operator to apply before the edge detection step*/

  int m_nbThread; /*!< Number of threads to use.*/

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
  std::vector<std::pair<unsigned int, float> > m_edgeCandidateAndGradient; /*!< Map that contains point image coordinates and corresponding gradient value.*/

  // // Hysteresis thresholding attributes
  float m_lowerThreshold; /*!< Lower threshold for the hysteresis step. If negative, it will be deduced
                               as from m_upperThreshold. */
  float m_lowerThresholdRatio; /*!< If the thresholds must be computed, the ratio of the upper threshold the lower
                                    threshold is equal: m_lowerThreshold = m_lowerThresholdRatio * m_upperThreshold. */
  float m_upperThreshold; /*!< Upper threshold for the hysteresis step.*/
  float m_upperThresholdRatio; /*!< If the thresholds must be computed, the ratio of pixels of the gradient image that
                                    must be lower than the upper threshold \b m_upperThreshold.*/

  // // Edge tracking attributes
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  rlim_t m_minStackSize; /*!< Minimum stack size, due to the recursivity used in this step of the algorithm.*/
#endif
  bool m_storeListEdgePoints; /*!< If true, the vector \b m_edgePointsList will contain the list of the edge points resulting from the whole algorithm.*/
  std::vector<unsigned int> m_activeEdgeCandidates; /*!< Vector that contains only the IDs of the edge candidates.*/
  static vpImage<EdgeType> m_edgePointsCandidates; /*!< Map that contains the strong edge points, i.e. the points for which we know for sure they are edge points,
                                                and the weak edge points, i.e. the points for which we still must determine if they are actual edge points.*/
  vpImage<unsigned char> m_edgeMap; /*!< Final edge map that results from the whole Canny algorithm.*/
  std::vector<vpImagePoint> m_edgePointsList; /*!< List of the edge points that belong to the final edge map.*/
  const vpImage<bool> *mp_mask; /*!< Mask that permits to consider only the pixels for which the mask is true.*/

  float getGradientOrientation(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &iter);

  void getInterpolWeightsAndOffsets(const float &gradientOrientation, float &alpha, float &beta, const int &nbCols,
                                    int &dRowGradAlpha, int &dRowGradBeta, int &dColGradAlpha, int &dColGradBeta);

  float getManhattanGradient(const vpImage<float> &dIx, const vpImage<float> &dIy, const int &iter);

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
   * \brief Perform the steps:
   * * 3 := edge-thining
   * * 4 := hysteresis thresholding
   * * 5 := edge tracking
   *
   * \param[in] lowerThreshold The lower threshold for the hysteresis thresholding.
   * \param[in] upperThreshold The upper threshold for the hysteresis thresholding.
   */
  void step3to5(const unsigned int &height, const unsigned int &width, const float &lowerThreshold, const float &upperThreshold);

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
  bool recursiveSearchForStrongEdge(const unsigned int &coordinates);

  /**
   * \brief Perform edge tracking.
   * \details For each weak edge, we will recursively check if they are 8-connected to a strong edge point.
   * If so, the weak edge will be saved in \b m_strongEdgePoints and will be kept in the final edge map.
   * Otherwise, the edge point will be discarded.
   */
  void performEdgeTracking();
  //@}
};

#ifdef VISP_HAVE_NLOHMANN_JSON
inline void from_json(const nlohmann::json &j, vpCannyEdgeDetection &detector)
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
  detector.m_nbThread = j.value("nbThread", detector.m_nbThread);
  detector.reinit();
}

inline void to_json(nlohmann::json &j, const vpCannyEdgeDetection &detector)
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
          {"upperThresholdRatio", detector.m_upperThresholdRatio},
          {"nbThread", detector.m_nbThread}
  };
}
#endif

END_VISP_NAMESPACE
#endif
