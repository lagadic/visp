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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace tutorial
{
inline void log(std::ostream &os, const std::string &filename, const std::string &funName, const std::string &arrayName, const VISP_NAMESPACE_ADDRESSING vpArray2D<double> &array, const unsigned int &level = 0)
{
  os << "[" << filename << "::" << funName << "] ";
  for (unsigned int i = 0; i < level; ++i) {
    os << "\t";
  }
  os << arrayName << ":=" << std::endl;
  for (unsigned int r = 0; r < array.getRows(); ++r) {
    for (unsigned int i = 0; i < level; ++i) {
      os << "\t";
    }
    os << "[";
    for (unsigned int c = 0; c < array.getCols() - 1; ++c) {
      os << std::setprecision(3) << std::scientific << array[r][c] << "\t; ";
    }
    os << array[r][array.getCols() - 1] << "]\n";
  }
  os << std::flush;
}

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
  double m_ratioSaltPepperNoise; /*!< Ratio of noise points to introduce in the addSaltAndPepperNoise function.*/
  unsigned int m_degree; //!< Degree for the polynomials.

  /// Images and displays parameters
  VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> m_I_orig; /*!< The color image read from the file.*/
  VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> m_I_segmented; /*!< The segmented color image resulting from HSV segmentation.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_mask; /*!< A binary mask where 255 means that a pixel belongs to the HSV range delimited by the HSV thresholds.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_Iskeleton; /*!< The image resulting from the skeletonization of the mask.*/
  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_IskeletonNoisy; /*!< The image resulting from the skeletonization of the mask, to which is added some salt and pepper noise.*/
#if defined(VISP_HAVE_DISPLAY)
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayOrig;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySegmented;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayNoisy;
#endif

  /// Particle filter parameters
  double m_pfMaxDistanceForLikelihood; /*!< Maximum tolerated distance for the likelihood evaluation.*/
  unsigned int m_pfN; /*!< Number of particles for the particle filter.*/
  std::vector<double> m_pfRatiosAmpliMax; /*!< The ratio of the initial guess the maximum amplitude of noise on each coefficient of the parabola.*/
  long m_pfSeed; /*!< The seed for the particle filter. A negative value will use the current timestamp.*/
  int m_pfNbThreads; /*!< Number of threads the Particle filter should use.*/

  vpTutoCommonData()
    : m_seqFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("data", "color_image_%04d.png"))
    , m_hsvFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("calib", "hsv-thresholds.yml"))
    , m_stepbystep(true)
    , m_ratioSaltPepperNoise(0.15)
    , m_degree(2)
    , m_pfMaxDistanceForLikelihood(40)
    , m_pfN(300)
    , m_pfRatiosAmpliMax({ 0.25, 0.25, 0.25 })
    , m_pfSeed(4221)
    , m_pfNbThreads(-1)
  { }

  /**
   * \brief Print the help about the program optional parameters.
   *
   * \param[in] softName The name of the program.
   */
  inline void printHelp(const char *softName)
  {
    std::cout << "\nSYNOPSIS " << std::endl
      << softName
      << " [--video <input video>] [--hsv-thresholds <filename.yml>] [--noise <ratio>]" << std::endl
      << " [--degree <uint>]" << std::endl
      << " [--max-distance-likelihood <double>] [-N, --nb-particles <uint>] [--seed <int>] [--nb-threads <int>] [--state-noise-ratio <ratio>]" << std::endl
      << " [--help,-h]"
      << std::endl;
    std::cout << "\nOPTIONS " << std::endl
      << " [General params]" << std::endl
      << "  --video <input video>" << std::endl
      << "    Name of the input video filename." << std::endl
      << "    If name is set to \"generate-simulated\" a simulated image is generated." << std::endl
      << "    Example: --video " << this->m_seqFilename << std::endl
      << std::endl
      << "  --hsv-thresholds <filename.yaml>" << std::endl
      << "    Path to a yaml filename that contains H <min,max>, S <min,max>, V <min,max> threshold values." << std::endl
      << "    For an example, have a look to the file \"" << this->m_hsvFilename << "\"" << std::endl
      << std::endl
      << "  --noise <ratio, [0; 1.[ >" << std::endl
      << "    Ratio of noisy points added to the image resulting from the skeletonization of the segmented image, to simulate sensor noise." << std::endl
      << "    Default = " << this->m_ratioSaltPepperNoise << std::endl
      << std::endl
      << "  --degree <uint>" << std::endl
      << "    Choose the degree of the polynomials to use." << std::endl
      << "    Default = " << this->m_degree << std::endl
      << std::endl
      << std::endl
      << " [PF params]" << std::endl
      << "  --max-distance-likelihood" << std::endl
      << "    Maximum mean square distance between a particle with the measurements." << std::endl
      << "    Above this value, the likelihood of the particle is 0." << std::endl
      << "    NOTE: M-estimation is used to make the likelihood function robust against outliers." << std::endl
      << "    Default: " << m_pfMaxDistanceForLikelihood << std::endl
      << std::endl
      << "  -N, --nb-particles" << std::endl
      << "    Number of particles of the Particle Filter." << std::endl
      << "    Default: " << m_pfN << std::endl
      << std::endl
      << "  --seed" << std::endl
      << "    Seed to initialize the Particle Filter." << std::endl
      << "    Use a negative value makes to use the current timestamp instead." << std::endl
      << "    Default: " << m_pfSeed << std::endl
      << std::endl
      << "  --nb-threads" << std::endl
      << "    Set the number of threads to use in the Particle Filter (only if OpenMP is available)." << std::endl
      << "    Use a negative value to use the maximum number of threads instead." << std::endl
      << "    Default: " << m_pfNbThreads << std::endl
      << std::endl
      << "  --state-noise-ratio <ratio>" << std::endl
      << "    Ratio of the initial guess of the curve coefficients to use as maximal amplitude of the noise added to the particles." << std::endl
      << "    Default: " << m_pfRatiosAmpliMax[0] << std::endl
      << "  --help, -h" << std::endl
      << "    Display this helper message." << std::endl
      << std::endl;
  }

  /**
   * \brief Initialize the program data from the command line arguments.
   *
   * \param[in] argc The number of optional parameters.
   * \param[in] argv The values of the optional parameters.
   * \return int Initialization status. EXIT_FAILURE if there was a problem, EXIT_SUCCESS if printing the program
   * help was asked and SOFTWARE_CONTINUE if the initialization went well.
   */
  inline int init(const int &argc, const char *argv[])
  {
    // Parse the input arguments
    int i = 1;
    while (i < argc) {
      std::string argname(argv[i]);
      if ((argname == std::string("--video")) && ((i + 1) < argc)) {
        ++i;
        m_seqFilename = std::string(argv[i]);
      }
      else if ((argname == std::string("--hsv-thresholds")) && ((i + 1) < argc)) {
        ++i;
        m_hsvFilename = std::string(argv[i]);
      }
      else if ((argname == "--noise") && ((i + 1) < argc)) {
        ++i;
        m_ratioSaltPepperNoise = std::atof(argv[i]);
      }
      else if ((argname == std::string("--degree")) && ((i + 1) < argc)) {
        ++i;
        m_degree = std::atoi(argv[i]);
      }
      else if ((argname == "--max-distance-likelihood") && ((i+1) < argc)) {
        ++i;
        m_pfMaxDistanceForLikelihood = std::atof(argv[i]);
      }
      else if (((argname == "-N") || (argname == "--nb-particles")) && ((i+1) < argc)) {
        ++i;
        m_pfN = std::atoi(argv[i]);
      }
      else if ((argname == "--seed") && ((i+1) < argc)) {
        ++i;
        m_pfSeed = std::atoi(argv[i]);
      }
      else if ((argname == "--nb-threads") && ((i+1) < argc)) {
        ++i;
        m_pfNbThreads = std::atoi(argv[i]);
      }
      else if ((argname == "--state-noise-ratio") && ((i+1) < argc)) {
        ++i;
        m_pfRatiosAmpliMax[0] = std::atof(argv[i]);
      }
      else if ((argname == std::string("-h")) || (argname == std::string("--help"))) {
        vpTutoCommonData helpPrinter;
        helpPrinter.printHelp(argv[0]);
        return EXIT_SUCCESS;
      }
      else {
        std::cerr << "Unknown argument \"" << argname << "\"" << std::endl;
        return EXIT_FAILURE;
      }
      ++i;
    }

    // Ensure that the maximal amplitude vector is of correct size and values
    m_pfRatiosAmpliMax.resize(m_degree, m_pfRatiosAmpliMax[0]);

    // Load the HSV thresholds
    if (VISP_NAMESPACE_ADDRESSING vpColVector::loadYAML(m_hsvFilename, m_hsv_values)) {
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
    catch (const VISP_NAMESPACE_ADDRESSING vpException &e) {
      std::cout << e.getStringMessage() << std::endl;
      return EXIT_FAILURE;
    }

    m_I_segmented.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the segmented image to match the original image
    m_mask.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the binary mask that indicates which pixels are in the allowed HSV range.
    m_Iskeleton.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the edge-map.
    m_IskeletonNoisy.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the edge-map.

    // Init the displays
#if defined(VISP_HAVE_DISPLAY)
    const int horOffset = 20, vertOffset = 25;
    std::string skeletonTitle("Skeletonized image (");
    skeletonTitle += (m_ratioSaltPepperNoise == 0 ? "without" : std::to_string(static_cast<unsigned int>(m_ratioSaltPepperNoise * 100.)) + "%");
    skeletonTitle += " noise)";
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displayNoisy = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_IskeletonNoisy, 2 * horOffset + m_I_orig.getWidth(), 2 * vertOffset + m_I_orig.getHeight(), skeletonTitle);
#endif
    return SOFTWARE_CONTINUE;
  }

#ifdef VISP_HAVE_DISPLAY
  template<typename T>
  void displayLegend(const VISP_NAMESPACE_ADDRESSING vpImage<T> &I)
  {
    VISP_NAMESPACE_ADDRESSING vpImagePoint ip(20, 20);
    VISP_NAMESPACE_ADDRESSING vpImagePoint offset(20, 0);
    if (m_stepbystep) {
      VISP_NAMESPACE_ADDRESSING vpDisplay::displayText(I, ip, std::string("Left click to switch to next image"), VISP_NAMESPACE_ADDRESSING vpColor::red);
    }
    VISP_NAMESPACE_ADDRESSING vpDisplay::displayText(I, ip + offset, std::string("Middle click to switch to ") + (m_stepbystep ? std::string("video mode") : std::string("step-by-step mode")), VISP_NAMESPACE_ADDRESSING vpColor::red);
    VISP_NAMESPACE_ADDRESSING vpDisplay::displayText(I, ip + offset + offset, std::string("Right click to quit"), VISP_NAMESPACE_ADDRESSING vpColor::red);
  }

  template<typename T>
  bool manageClicks(const VISP_NAMESPACE_ADDRESSING vpImage<T> &I, bool &stepbystep)
  {
    VISP_NAMESPACE_ADDRESSING vpImagePoint ip;
    VISP_NAMESPACE_ADDRESSING vpMouseButton::vpMouseButtonType button;
    VISP_NAMESPACE_ADDRESSING vpDisplay::getClick(I, ip, button, stepbystep);
    if (button == VISP_NAMESPACE_ADDRESSING vpMouseButton::vpMouseButtonType::button3) {
      return false;
    }
    if (button == VISP_NAMESPACE_ADDRESSING vpMouseButton::vpMouseButtonType::button2) {
      stepbystep = stepbystep ^ true;
    }
    return true;
  }
#endif
}vpTutoCommonData;
}
#endif
#endif
#endif
