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
inline void log(std::ostream &os, const std::string &filename, const std::string &funName, const std::string &arrayName, const vpArray2D<double> &array, const unsigned int &level = 0)
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
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayOrig;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySegmented;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displaySkeleton;
  std::shared_ptr<VISP_NAMESPACE_ADDRESSING vpDisplay> m_displayNoisy;
#elif defined(VISP_HAVE_DISPLAY)
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displayOrig;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displaySegmented;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displaySkeleton;
  VISP_NAMESPACE_ADDRESSING vpDisplay *m_displayNoisy;
#endif
  /// Ransac parameters
  unsigned int m_ransacN; /*!< The number of points to use to build the model.*/
  unsigned int m_ransacK; /*!< The number of iterations.*/
  float m_ransacThresh; /*!< The threshold that indicates if a point fit the model or not.*/
  float m_ransacRatioInliers; /*!< Ratio of points that the model explain.*/

  /// Particle filter parameters
  double m_pfMaxDistanceForLikelihood; /*!< Maximum tolerated distance for the likelihood evaluation.*/
  unsigned int m_pfN; /*!< Number of particles for the particle filter.*/
  std::vector<double> m_pfRatiosAmpliMax; /*!< The ratio of the initial guess the maximum amplitude of noise on each coefficient of the parabola.*/
  long m_pfSeed; /*!< The seed for the particle filter. A negative value will use the current timestamp.*/
  int m_pfNbThreads; /*!< Number of threads the Particle filter should use.*/

  /// Simulation parameters
  double m_a; //!< To generate 2nd-degree polynomial simulated data.
  double m_b; //!< To generate 2nd-degree polynomial simulated data.
  double m_c; //!< To generate 2nd-degree polynomial simulated data.
  double m_a3; //!< To generate 3nd-degree polynomial simulated data.
  double m_b3; //!< To generate 3nd-degree polynomial simulated data.
  double m_c3; //!< To generate 3nd-degree polynomial simulated data.
  double m_d3; //!< To generate 3nd-degree polynomial simulated data.
  VISP_NAMESPACE_ADDRESSING vpColVector m_coeffsGT; //!< Ground truth coefficients when simulated data is used.
  bool m_useSimulated; //!< If true, use a generated polynomial curve instead of input images.

  void setGTCoeffs()
  {
    if (m_degree == 2) {
      m_coeffsGT.resize(3, 0.);
      m_coeffsGT[0] = m_c;
      m_coeffsGT[1] = m_b;
      m_coeffsGT[2] = m_a;
    }
    else if (m_degree ==3) {
      m_coeffsGT.resize(4, 0.);
      m_coeffsGT[0] = m_d3;
      m_coeffsGT[1] = m_c3;
      m_coeffsGT[2] = m_b3;
      m_coeffsGT[3] = m_a3;
    }
    else {
      m_coeffsGT.resize(m_degree + 1);
      m_coeffsGT[0] = 20.;
      for (unsigned int i = 1; i < m_degree + 1; ++i) {
        m_coeffsGT[i] = std::pow(0.005, i) * i;
      }
    }
    std::cout << "GT coeffs = " << m_coeffsGT.transpose() << std::endl;
  }

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

  /**
   * \brief Compute the v-coordinate of an image point based on the u-coordinate and the polynomial used
   * for the simulation.
   * \param[in] x The u-coordinate of the image point.
   * \return The corresponding v-coordinate.
   */
  double computeV(const double &u)
  {
    double v = 0.;
    if (m_degree == 2) {
      v = m_a * u * u + m_b * u + m_c;
    }
    else if (m_degree == 3) {
      v = m_a3 * u * u * u + m_b3 * u * u + m_c3 * u + m_d3;
    }
    else if (m_degree > 3) {
      for (unsigned int i = 0; i <= m_degree; ++i) {
        v += m_coeffsGT[i] * std::pow(u, i);
      }
    }
    else if (m_degree == 1) {
      v = 0.5 * u + 20;
    }
    return u;
  }

  /**
   * \brief Generate a simulated image based on the polynomial the user chose to use.
   */
  inline void generateSimulatedImage()
  {
    const unsigned int width = 600, height = 400;
    m_I_orig.resize(height, width, vpRGBa(0));
    VISP_NAMESPACE_ADDRESSING vpRect limits(VISP_NAMESPACE_ADDRESSING vpImagePoint(0., 0.), VISP_NAMESPACE_ADDRESSING vpImagePoint(height - 1, width - 1));
    VISP_NAMESPACE_ADDRESSING vpRGBa color(175, 175, 53);
    for (unsigned int uInt = 0; uInt < width; ++uInt) {
      double u = static_cast<double>(uInt);
      double v = computeV(u);
      vpImagePoint pt(v, u);
      if (limits.isInside(pt)) {
        int vInt = static_cast<int>(v);
        m_I_orig[vInt][uInt] = color;
      }
    }
  }

  vpTutoCommonData()
    : m_seqFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("data", "color_image_%04d.png"))
    , m_hsvFilename(VISP_NAMESPACE_ADDRESSING vpIoTools::createFilePath("calib", "hsv-thresholds.yml"))
    , m_stepbystep(true)
    , m_ratioSaltPepperNoise(0.15)
    , m_degree(2)
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
    , m_displayOrig(nullptr)
    , m_displaySegmented(nullptr)
    , m_displaySkeleton(nullptr)
    , m_displayNoisy(nullptr)
#endif
    , m_ransacN(10)
    , m_ransacK(10000)
    , m_ransacThresh(1600.)
    , m_ransacRatioInliers(0.5f)
    , m_pfMaxDistanceForLikelihood(40)
    , m_pfN(300)
    , m_pfRatiosAmpliMax({ 0.25, 0.25, 0.25 })
    , m_pfSeed(4221)
    , m_pfNbThreads(-1)
    , m_useSimulated(false)
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

    if (m_displayNoisy != nullptr) {
      delete m_displayNoisy;
      m_displayNoisy = nullptr;
    }
  }
#endif

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
      << " [--degree <uint>] [--curve <x0 y0 x1 y1>]" << std::endl
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
      << "  --curve <x0 y0 x1 y1>" << std::endl
      << "    For a 2nd degree polynomial, (x0, y0) is the inflexion point and (x1, y1) is a 2nd point of the curve." << std::endl
      << "    For a 3rd degree polynomial, (x0, y0) and (x1, y1) are the inflexion points." << std::endl
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
      else if ((argname == std::string("--curve")) && ((i + 4) < argc)) {
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

    if (m_ransacN < m_degree + 1) {
      // The number of points to use in the RANSAC to determine the polynomial coefficients
      // must be at least equal to m_degree + 1, otherwise the problem is under-constrained
      m_ransacN = 2 * (m_degree + 1);
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
      m_useSimulated = true;
      std::cout << "Degree of the polynomial = " << m_degree << std::endl;
      setGTCoeffs();
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
    m_IskeletonNoisy.resize(m_I_orig.getHeight(), m_I_orig.getWidth()); // Resize the edge-map.

    // Init the displays
    const int horOffset = 20, vertOffset = 20;
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displaySkeleton = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_Iskeleton, horOffset, 2 * vertOffset + m_I_orig.getHeight(), "Skeletonized image");
    m_displayNoisy = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::createDisplay(m_IskeletonNoisy, 2 * horOffset + m_I_orig.getWidth(), 2 * vertOffset + m_I_orig.getHeight(), "Noisy skeletonized image");
#elif defined(VISP_HAVE_DISPLAY)
    m_displayOrig = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_orig, horOffset, vertOffset, "Original image");
    m_displaySegmented = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_I_segmented, 2 * horOffset + m_I_orig.getWidth(), vertOffset, "Segmented image");
    m_displaySkeleton = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_Iskeleton, horOffset, 2 * vertOffset + m_I_orig.getHeight(), "Skeletonized image");
    m_displayNoisy = VISP_NAMESPACE_ADDRESSING vpDisplayFactory::allocateDisplay(m_IskeletonNoisy, 2 * horOffset + m_I_orig.getWidth(), 2 * vertOffset + m_I_orig.getHeight(), "Noisy skeletonized image");
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
