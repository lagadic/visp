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

//! \example tutorial-pf-curve-fitting-lms.cpp

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMouseButton.h>
#include <visp3/core/vpTime.h>

#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#endif

#include "vpTutoCommonData.h"
#include "vpTutoMeanSquareFitting.h"
#include "vpTutoParabolaModel.h"
#include "vpTutoSegmentation.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
int main(const int argc, const char *argv[])
{
  tutorial::vpTutoCommonData data;
  int returnCode = data.init(argc, argv);
  if (returnCode != tutorial::vpTutoCommonData::SOFTWARE_CONTINUE) {
    return returnCode;
  }
  tutorial::vpTutoMeanSquareFitting lmsFitter(data.m_degree, data.m_I_orig.getHeight(), data.m_I_orig.getWidth());
  const unsigned int vertOffset = static_cast<unsigned int>(data.m_legendOffset.get_i());
  const unsigned int horOffset = static_cast<unsigned int>(data.m_ipLegend.get_j());
  const unsigned int legendLmsVert = data.m_I_orig.getHeight() - 2 * vertOffset;
  const unsigned int legendLmsHor = horOffset;

  //! [Init_plot]
#ifdef VISP_HAVE_DISPLAY
  unsigned int plotHeight = 350, plotWidth = 350;
  int plotXpos = static_cast<int>(data.m_legendOffset.get_u());
  int plotYpos = static_cast<int>(data.m_I_orig.getHeight() + 4. * data.m_legendOffset.get_v());
  vpPlot plot(1, plotHeight, plotWidth, plotXpos, plotYpos, "Root mean-square error");
  plot.initGraph(0, 1);
  plot.setLegend(0, 0, "LMS estimator");
  plot.setColor(0, 0, vpColor::gray);
#endif
//! [Init_plot]

  bool run = true;
  unsigned int nbIter = 0;
  double  meanDtLMS = 0.;
  double  meanRootMeanSquareErrorLMS = 0.;
  while (!data.m_grabber.end() && run) {
    //! [Measurements_extraction]
    std::cout << "Iter " << nbIter << std::endl;
    data.m_grabber.acquire(data.m_I_orig);

    // Perform color segmentation
    tutorial::performSegmentationHSV(data);

    /// Extracting the skeleton of the mask
    std::vector<vpImagePoint> edgePoints = tutorial::extractSkeleton(data);

    /// Simulate sensor noise
    std::vector<vpImagePoint> noisyEdgePoints = tutorial::addSaltAndPepperNoise(edgePoints, data);
    //! [Measurements_extraction]

#ifdef VISP_HAVE_DISPLAY
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::display(data.m_I_segmented);
    vpDisplay::display(data.m_IskeletonNoisy);
#endif

    /// Fit using least-square
    double tLms = vpTime::measureTimeMs();
    //! [LMS_interpolation]
    lmsFitter.fit(noisyEdgePoints);
    //! [LMS_interpolation]
    double dtLms = vpTime::measureTimeMs() - tLms;
    double lmsRootMeanSquareError = lmsFitter.evaluate(edgePoints);
    std::cout << "  [Least-Mean Square method] " << std::endl;
    std::cout << "    Coeffs = [" << lmsFitter.getCoeffs().transpose() << " ]" << std::endl;
    std::cout << "    Root Mean Square Error = " << lmsRootMeanSquareError << " pixels" << std::endl;
    std::cout << "    Fitting duration = " << dtLms << " ms" << std::endl;
    meanDtLMS += dtLms;
    meanRootMeanSquareErrorLMS += lmsRootMeanSquareError;

#ifdef VISP_HAVE_DISPLAY
    // Update image overlay
    lmsFitter.display<unsigned char>(data.m_IskeletonNoisy, vpColor::gray, legendLmsVert, legendLmsHor);

    // Update plot
    plot.plot(0, 0, nbIter, lmsRootMeanSquareError);
    // Display the images with overlayed info
    data.displayLegend(data.m_I_orig);
    vpDisplay::flush(data.m_I_orig);
    vpDisplay::flush(data.m_I_segmented);
    vpDisplay::flush(data.m_IskeletonNoisy);
    run = data.manageClicks(data.m_I_orig, data.m_stepbystep);
#endif
    ++nbIter;
  }

  double iterAsDouble = static_cast<double>(nbIter);
  std::cout << std::endl << std::endl << "-----[Statistics summary]-----" << std::endl;
  std::cout << "  [LMS method] " << std::endl;
  std::cout << "    Average Root Mean Square Error = " << meanRootMeanSquareErrorLMS / iterAsDouble << " pixels" << std::endl;
  std::cout << "    Average fitting duration = " << meanDtLMS / iterAsDouble << " ms" << std::endl;

#ifdef VISP_HAVE_DISPLAY
  if (data.m_grabber.end() && (!data.m_stepbystep)) {
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "End of sequence reached. Click to exit.", data.m_colorLegend);

    /// Update the display
    vpDisplay::flush(data.m_I_orig);

    /// Get the user input
    vpDisplay::getClick(data.m_I_orig, true);
  }
#endif
  return 0;
}
#else
int main()
{
  std::cerr << "ViSP must be compiled with C++ standard >= C++11 to use this tutorial." << std::endl;
  std::cerr << "ViSP must also have a 3rd party enabling display features, such as X11 or OpenCV." << std::endl;
  return EXIT_FAILURE;
}
#endif
