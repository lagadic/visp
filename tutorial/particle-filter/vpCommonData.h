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
typedef struct vpCommonData
{
  static const int SOFTWARE_CONTINUE = 4221;
  std::string m_seqFilename; /*!< Sequence filenames, such as I%04d.png*/
  VISP_NAMESPACE_ADDRESSING vpVideoReader m_grabber; /*!< Video grabber from stored files.*/
  std::string m_hsvFilename; /*!< Filename of the YAML file that contains the HSV thresholds.*/
  VISP_NAMESPACE_ADDRESSING vpColVector m_hsv_values; /*!< Vector that contains the lower and upper limits of the HSV thresholds.*/
  VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> m_I_orig; /*!< The color image read from the file.*/
  VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> m_I_segmented; /*!< The segmented color image resulting from HSV segmentation.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_mask; /*!< A binary mask where 255 means that a pixel belongs to the HSV range delimited by the HSV thresholds.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_Icanny; /*!< The edge-map resulting from the mask.*/
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayOrig;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySegmented;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayCanny;
#else
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displayOrig;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displaySegmented;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displayCanny;
#endif
  int m_cannyGfKernelSize; /*!< The kernel size of the Gaussian filter.*/
  float m_cannyGfStdev; /*!< The standard deviation of the Gaussian filter.*/
  unsigned int m_cannyGradAperture; /*!< The kernel size for the computation of the gradient.*/
  float m_cannyLt; /*!< The lower threshold for the Canny edge-detection. Negative value to compute it automatically.*/
  float m_cannyUpperT; /*!< The upper threshold for the Canny edge-detection. Negative value to compute it automatically.*/
  float m_cannyLtr; /*!< The ratio for the automatic computation of the lower threshold.*/
  float m_cannyUpperTr; /*!< The ratio for the automatic computation of the upper threshold.*/
  VISP_NAMESPACE_ADDRESSING vpImageFilter::vpCannyFilteringAndGradientType m_cannyGradType; /*!< The type of gradient filter.*/
  unsigned int m_ransacN; /*!< The number of points to use to build the model.*/
  unsigned int m_ransacK; /*!< The number of iterations.*/
  float m_ransacThresh; /*!< The threshold that indicates if a point fit the model or not.*/
  float m_ransacRatioInliers; /*!< Ratio of points that the model explain.*/

  vpCommonData()
    : m_seqFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("data", "color_image_%04d.png"))
    , m_hsvFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("calib", "hsv-thresholds.yml"))
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    , m_displayOrig(nullptr)
    , m_displaySegmented(nullptr)
    , m_displayCanny(nullptr)
#endif
    , m_cannyGfKernelSize(5)
    , m_cannyGfStdev(1.f)
    , m_cannyGradAperture(3)
    , m_cannyLt(-1.f)
    , m_cannyUpperT(-1.f)
    , m_cannyLtr(0.6f)
    , m_cannyUpperTr(0.8f)
    , m_cannyGradType(VISP_NAMESPACE_ADDRESSING vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)
    , m_ransacN(10)
    , m_ransacK(10000)
    , m_ransacThresh(1600.)
    , m_ransacRatioInliers(0.5f)
  { }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  ~vpCommonData()
  {
    if (m_displayOrig != nullptr) {
      delete m_displayOrig;
      m_displayOrig = nullptr;
    }
    if (m_displaySegmented != nullptr) {
      delete m_displaySegmented;
      m_displaySegmented = nullptr;
    }

    if (m_displayCanny != nullptr) {
      delete m_displayCanny;
      m_displayCanny = nullptr;
    }
  }
#endif

  inline void printHelp(const char *softName)
  {
    std::cout << "\nSYNOPSIS " << std::endl
      << softName
      << " [--video <input video>]"
      << " [--hsv-thresholds <filename.yml>]"
      << " [--help,-h]"
      << std::endl;
    std::cout << "\nOPTIONS " << std::endl
      << "  --video <input video>" << std::endl
      << "    Name of the input video filename." << std::endl
      << "    Example: --video " << this->m_seqFilename << std::endl
      << std::endl
      << "  --hsv-thresholds <filename.yaml>" << std::endl
      << "    Path to a yaml filename that contains H <min,max>, S <min,max>, V <min,max> threshold values." << std::endl
      << "    For an example, have a look to the file \"" << this->m_hsvFilename << "\"" << std::endl
      << std::endl
      << "  --help, -h" << std::endl
      << "    Display this helper message." << std::endl
      << std::endl;
  }

  inline int init(const int &argc, const char *argv[])
  {
    // Parse the input arguments
    int i = 1;
    while (i < argc) {
      std::string argname(argv[i]);
      if (argname == std::string("--video")) {
        ++i;
        m_seqFilename = std::string(argv[i]);
      }
      else if (argname == std::string("--hsv-thresholds")) {
        ++i;
        m_hsvFilename = std::string(argv[i]);
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

    // Open the sequence of images
    try {
      m_grabber.setFileName(m_seqFilename);
      m_grabber.open(m_I_orig);
    }
    catch (const vpException &e) {
      std::cout << e.getStringMessage() << std::endl;
      return EXIT_FAILURE;
    }
    m_I_segmented.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the segmented image to match the original image
    m_mask.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the binary mask that indicates which pixels are in the allowed HSV range.
    m_Icanny.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the edge-map.

    // Init the displays
    const int horOffset = 20, vertOffset = 20;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displayCanny = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_Icanny, horOffset, 2 * vertOffset + m_I_orig.getHeight(), "Edge-map");
#else
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displayCanny = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_Icanny, horOffset, 2 * vertOffset + m_I_orig.getHeight(), "Edge-map");
#endif
    return SOFTWARE_CONTINUE;
  }
}vpCommonData;
}
#endif
