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
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayOrig;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySegmented;
#else
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displayOrig;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displaySegmented;
#endif

  vpCommonData()
    : m_seqFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("data", "color_image_%04d.png"))
    , m_hsvFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("calib", "hsv-thresholds.yml"))
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    , m_displayOrig(nullptr)
    , m_displaySegmented(nullptr)
#endif
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
    m_I_segmented = m_I_orig; // Resize the segmented image to match the original image

    // Init the displays
    const int horOffset = 20, vertOffset = 20;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_segmented, 3 * horOffset, vertOffset, "Segmented image");
#else
    // m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    // m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_segmented, 3 * horOffset, vertOffset, "Segmented image");
#endif
    return SOFTWARE_CONTINUE;
  }
}vpCommonData;
}
#endif
