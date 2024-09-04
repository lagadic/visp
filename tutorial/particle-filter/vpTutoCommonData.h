/****************************************************************************
 *
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
#ifndef VP_COMMMON_DATA_H
#define VP_COMMMON_DATA_H
#include <iostream>
#include <string>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoReader.h>

namespace tutorial
{
typedef struct vpTutoCommonData
{
  static const int SOFTWARE_CONTINUE = 4221;
  const VISP_NAMESPACE_ADDRESSING vpColor m_colorLegend = VISP_NAMESPACE_ADDRESSING vpColor::red;
  const VISP_NAMESPACE_ADDRESSING vpImagePoint m_ipLegend = VISP_NAMESPACE_ADDRESSING vpImagePoint(20, 20);
  const VISP_NAMESPACE_ADDRESSING vpImagePoint m_legendOffset = VISP_NAMESPACE_ADDRESSING vpImagePoint(20, 0);
  std::string m_seqFilename; /*!< Sequence filenames, such as I%04d.png*/
  VISP_NAMESPACE_ADDRESSING vpVideoReader m_grabber; /*!< Video grabber from stored files.*/
  std::string m_hsvFilename; /*!< Filename of the YAML file that contains the HSV thresholds.*/
  VISP_NAMESPACE_ADDRESSING vpColVector m_hsv_values; /*!< Vector that contains the lower and upper limits of the HSV thresholds.*/
  bool m_stepbystep; /*!< If true, the frames are treated in a step by step mode, otherwise the frames are treated as a video.*/
  VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> m_I_orig; /*!< The color image read from the file.*/
  VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> m_I_segmented; /*!< The segmented color image resulting from HSV segmentation.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_mask; /*!< A binary mask where 255 means that a pixel belongs to the HSV range delimited by the HSV thresholds.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_Iskeleton; /*!< The image resulting from the skeletonization of the mask.*/
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayOrig;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySegmented;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySkeleton;
#elif defined(VISP_HAVE_DISPLAY)
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displayOrig;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displaySegmented;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displaySkeleton;
#endif
  unsigned int m_ransacN; /*!< The number of points to use to build the model.*/
  unsigned int m_ransacK; /*!< The number of iterations.*/
  float m_ransacThresh; /*!< The threshold that indicates if a point fit the model or not.*/
  float m_ransacRatioInliers; /*!< Ratio of points that the model explain.*/

  double m_pfMaxDistanceForLikelihood; /*!< Maximum tolerated distance for the likelihood evaluation.*/
  unsigned int m_pfN; /*!< Number of particles for the particle filter.*/
  double m_pfRatioAmpliMaxA; /*!< The ratio of the initial guess the maximum amplitude of noise on the a coefficient of the parabola v = a u^2 + b u + c*/
  double m_pfRatioAmpliMaxB; /*!< The ratio of the initial guess the maximum amplitude of noise on the b coefficient of the parabola v = a u^2 + b u + c*/
  double m_pfRatioAmpliMaxC; /*!< The ratio of the initial guess the maximum amplitude of noise on the v coefficient of the parabola v = a u^2 + b u + c*/
  long m_pfSeed; /*!< The seed for the particle filter. A negative value will use the current timestamp.*/
  int m_pfNbThreads; /*!< Number of threads the Particle filter should use.*/

  double m_a; //!< To generate 2nd-degree polynomial simulated data.
  double m_b; //!< To generate 2nd-degree polynomial simulated data.
  double m_c; //!< To generate 2nd-degree polynomial simulated data.
  double m_a3; //!< To generate 3nd-degree polynomial simulated data.
  double m_b3; //!< To generate 3nd-degree polynomial simulated data.
  double m_c3; //!< To generate 3nd-degree polynomial simulated data.
  double m_d3; //!< To generate 3nd-degree polynomial simulated data.
  unsigned int m_degree; //!< Degree for the polynomials.

  /**
   * \brief Compute the coefficients of the 2nd degree curve for the simulated data.
   *
   * \param[in] x0 Horizontal coordinate of the inflexion point.
   * \param[in] y0 Vertical coordinate of the inflexion point.
   * \param[in] x1 Horizontal coordinate of another point of the curve.
   * \param[in] y1 Vertical coordinate of another point of the curve.
   */
  void computeABC(const double &x0, const double &y0, const double &x1, const double &y1)
  {
    m_b = (y1 - y0)/(-0.5*(x1 * x1/x0) + x1 -0.5 * x0);
    m_a = -m_b / (2. * x0);
    m_c = y0 - 0.5 * m_b * x0;
  }

  /**
   * \brief Compute the coefficients of the 2nd degree curve for the simulated data.
   *
   * \param[in] x0 Horizontal coordinate of the inflexion point.
   * \param[in] y0 Vertical coordinate of the inflexion point.
   * \param[in] x1 Horizontal coordinate of another point of the curve.
   * \param[in] y1 Vertical coordinate of another point of the curve.
   */
  void computeABCD(const double &x0, const double &y0, const double &x1, const double &y1)
  {
    double factorA = -2. / (3. * (x1 + x0));
    double factorC = -1. * ((-2. * std::pow(x0, 2))/(x1 + x0) + 2 * x0);
    m_b3 = (y1 - y0)/(factorA * (std::pow(x1, 3) - std::pow(x0, 3)) + (std::pow(x1, 2) - std::pow(x0, 2)) + (x1 - x0) * factorC);
    m_a3 = factorA * m_b3;
    m_c3 = factorC * m_b3;
    m_d3 = y0-(m_a3 * std::pow(x0, 3) + m_b3 * std::pow(x0, 2) + m_c3 * x0);
  }

  double computeY(const double &x)
  {
    double y = 0.;
    if (m_degree == 2) {
      y = m_a * x * x + m_b * x + m_c;
    }
    else if (m_degree == 3) {
      y = m_a3 * x * x * x + m_b3 * x * x + m_c3 * x + m_d3;
    }
    return y;
  }

  vpTutoCommonData()
    : m_seqFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("data", "color_image_%04d.png"))
    , m_hsvFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("calib", "hsv-thresholds.yml"))
    , m_stepbystep(true)
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
    , m_displayOrig(nullptr)
    , m_displaySegmented(nullptr)
    , m_displaySkeleton(nullptr)
#endif
    , m_ransacN(10)
    , m_ransacK(10000)
    , m_ransacThresh(1600.)
    , m_ransacRatioInliers(0.5f)
    , m_pfMaxDistanceForLikelihood(40)
    , m_pfN(300)
    , m_pfRatioAmpliMaxA(0.25)
    , m_pfRatioAmpliMaxB(0.25)
    , m_pfRatioAmpliMaxC(0.25)
    , m_pfSeed(4221)
    , m_pfNbThreads(-1)
    , m_degree(2)
  {
    double x0 = 300., y0 = 20., x1 = 20., y1 = 400.;
    computeABC(x0, y0, x1, y1);
    computeABCD(x0, y0, x1, y1);
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
  ~vpTutoCommonData()
  {
    if (m_displayOrig != nullptr) {
      delete m_displayOrig;
      m_displayOrig = nullptr;
    }
    if (m_displaySegmented != nullptr) {
      delete m_displaySegmented;
      m_displaySegmented = nullptr;
    }

    if (m_displaySkeleton != nullptr) {
      delete m_displaySkeleton;
      m_displaySkeleton = nullptr;
    }
  }
#endif

  inline void printHelp(const char *softName)
  {
    std::cout << "\nSYNOPSIS " << std::endl
      << softName
      << " [--video <input video>]"
      << " [--hsv-thresholds <filename.yml>]"
      << " [--degree {2, 3}]"
      << " [--curve <x0 y0 x1 y1>]"
      << " [--help,-h]"
      << std::endl;
    std::cout << "\nOPTIONS " << std::endl
      << "  --video <input video>" << std::endl
      << "    Name of the input video filename." << std::endl
      << "    If name is set to \"generate-simulated\" a simulated image is generated." << std::endl
      << "    Example: --video " << this->m_seqFilename << std::endl
      << std::endl
      << "  --hsv-thresholds <filename.yaml>" << std::endl
      << "    Path to a yaml filename that contains H <min,max>, S <min,max>, V <min,max> threshold values." << std::endl
      << "    For an example, have a look to the file \"" << this->m_hsvFilename << "\"" << std::endl
      << std::endl
      << "  --degree {2, 3}" << std::endl
      << "    Choose the degree of the polynomials to use." << std::endl
      << "    Accepted values are 2 or 3, default = " << this->m_degree << std::endl
      << std::endl
      << "  --curve <x0 y0 x1 y1>" << std::endl
      << "    For a 2nd degree polynomial, (x0, y0) is the inflexion point and (x1, y1) is a 2nd point of the curve." << std::endl
      << "    For a 3rd degree polynomial, (x0, y0) and (x1, y1) are the inflexion points." << std::endl
      << std::endl
      << "  --help, -h" << std::endl
      << "    Display this helper message." << std::endl
      << std::endl;
  }

  inline void generateSimulatedImage()
  {
    const unsigned int width = 600, height = 400;
    m_I_orig.resize(height, width, vpRGBa(0));
    VISP_NAMESPACE_ADDRESSING vpRect limits(VISP_NAMESPACE_ADDRESSING vpImagePoint(0., 0.), VISP_NAMESPACE_ADDRESSING vpImagePoint(height - 1, width - 1));
    VISP_NAMESPACE_ADDRESSING vpRGBa color(175, 175, 53);
    for (unsigned int u = 0; u < width; ++u) {
      double x = static_cast<double>(u);
      double y = computeY(x);
      vpImagePoint pt(y, x);
      if (limits.isInside(pt)) {
        m_I_orig[static_cast<int>(y)][u] = color;
      }
    }
  }

  inline int init(const int &argc, const char *argv[])
  {
    // Parse the input arguments
    int i = 1;
    while (i < argc) {
      std::string argname(argv[i]);
      if (argname == std::string("--video") && ((i + 1) < argc)) {
        ++i;
        m_seqFilename = std::string(argv[i]);
      }
      else if (argname == std::string("--hsv-thresholds") && ((i + 1) < argc)) {
        ++i;
        m_hsvFilename = std::string(argv[i]);
      }
      else if (argname == std::string("--curve") && ((i + 4) < argc)) {
        ++i;
        double x0 = std::atof(argv[i]);
        ++i;
        double y0 = std::atof(argv[i]);
        ++i;
        double x1 = std::atof(argv[i]);
        ++i;
        double y1 = std::atof(argv[i]);
        computeABC(x0, y0, x1, y1);
        computeABCD(x0, y0, x1, y1);
      }
      else if (argname == std::string("--degree") && ((i + 1) < argc)) {
        ++i;
        m_degree = std::atoi(argv[i]);
      }
      else if ((argname == std::string("-h")) || (argname == std::string("--help"))) {
        printHelp(argv[0]);
        return EXIT_SUCCESS;
      }
      else {
        std::cerr << "Unknown argument \"" << argname << "\"" << std::endl;
        return EXIT_FAILURE;
      }
      ++i;
    }

    // Load the HSV thresholds
    if (vpColVector::loadYAML(m_hsvFilename, m_hsv_values)) {
      std::cout << "Load HSV threshold values from " << m_hsvFilename << std::endl;
      std::cout << "HSV low/high values: " << m_hsv_values.t() << std::endl;
    }
    else {
      std::cout << "ERROR: unable to load HSV thresholds values from " << m_hsvFilename << std::endl;
      return EXIT_FAILURE;
    }

    if (m_seqFilename.find("generate-simulated") != std::string::npos) {
      std::cout << "Degree of the polynomial = " << m_degree << std::endl;
      generateSimulatedImage();
    }
    else {
    // Open the sequence of images
      try {
        m_grabber.setFileName(m_seqFilename);
        m_grabber.open(m_I_orig);
      }
      catch (const vpException &e) {
        std::cout << e.getStringMessage() << std::endl;
        return EXIT_FAILURE;
      }
    }
    m_I_segmented.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the segmented image to match the original image
    m_mask.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the binary mask that indicates which pixels are in the allowed HSV range.
    m_Iskeleton.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the edge-map.

    // Init the displays
    const int horOffset = 20, vertOffset = 20;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displaySkeleton = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_Iskeleton, horOffset, 2 * vertOffset + m_I_orig.getHeight(), "Skeletonized image");
#elif defined(VISP_HAVE_DISPLAY)
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displaySkeleton = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_Iskeleton, horOffset, 2 * vertOffset + m_I_orig.getHeight(), "Skeletonized image");
#endif
    return SOFTWARE_CONTINUE;
  }

#ifdef VISP_HAVE_DISPLAY
  template<typename T>
  void displayLegend(const vpImage<T> &I)
  {
    vpImagePoint ip(20, 20);
    vpImagePoint offset(20, 0);
    if (m_stepbystep) {
      vpDisplay::displayText(I, ip, std::string("Left click to switch to next image"), vpColor::red);
    }
    vpDisplay::displayText(I, ip + offset, std::string("Middle click to switch to ") + (m_stepbystep ? std::string("video mode") : std::string("step-by-step mode")), vpColor::red);
    vpDisplay::displayText(I, ip + offset + offset, std::string("Right click to quit"), vpColor::red);
  }

  template<typename T>
  bool manageClicks(const vpImage<T> &I, bool &stepbystep)
  {
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button;
    vpDisplay::getClick(I, ip, button, stepbystep);
    if (button == vpMouseButton::vpMouseButtonType::button3) {
      return false;
    }
    if (button == vpMouseButton::vpMouseButtonType::button2) {
      stepbystep = stepbystep ^ true;
    }
    return true;
  }
#endif
  }vpTutoCommonData;
    }
#endif
