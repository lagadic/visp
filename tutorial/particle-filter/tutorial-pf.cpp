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

//! \example tutorial-pf.cpp

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpTime.h>

#include "vpCommonData.h"
#include "vpTutoMeanSquareFitting.h"
#include "vpTutoSegmentation.h"

int main(const int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using VISP_NAMESPACE_NAME;
#endif
  tutorial::vpCommonData data;
  int returnCode = data.init(argc, argv);
  if (returnCode != tutorial::vpCommonData::SOFTWARE_CONTINUE) {
    return returnCode;
  }

  const bool storeEdgePointsList = true;
  vpCannyEdgeDetection edgeDetector(data.m_cannyGfKernelSize, data.m_cannyGfStdev, data.m_cannyGradAperture, data.m_cannyLt,
                                    data.m_cannyUpperT, data.m_cannyLtr, data.m_cannyUpperTr, data.m_cannyGradType, storeEdgePointsList);
  tutorial::vpTutoMeanSquareFitting lmsFitter;
  const double period = 33.; // 33ms period, i.e. 30Hz
  const unsigned int vertOffset = 20;
  const unsigned int horOffset = 20;
  const unsigned int legendLmsVert = data.m_I_orig.getHeight() - 3 * vertOffset;
  const unsigned int legendLmsHor = horOffset;
  while (!data.m_grabber.end()) {
    double t0 = vpTime::measureTimeMs();
    data.m_grabber.acquire(data.m_I_orig);
    tutorial::performSegmentationHSV(data);
#ifdef VISP_HAVE_DISPLAY
    // Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::display(data.m_I_segmented);
    vpDisplay::display(data.m_Icanny);
#endif
    // Computing the edge-map from the mask
    data.m_Icanny = edgeDetector.detect(data.m_mask);
    std::vector<vpImagePoint> edgePoints = edgeDetector.getEdgePointsList();

    /// Fit using least-square
    float lmsError = lmsFitter.fit(edgePoints);
    std::cout << "Average error using least-mean square method: " << lmsError << " pixels" << std::endl;
    lmsFitter.display(data.m_Icanny, vpColor::blue, legendLmsVert, legendLmsHor);

    ///TODO: PF
#ifdef VISP_HAVE_DISPLAY
    // Display the images with overlayed info
    vpDisplay::flush(data.m_I_orig);
    vpDisplay::flush(data.m_I_segmented);
    vpDisplay::flush(data.m_Icanny);
#endif
    vpTime::wait(t0, period);
  }
  return 0;
}
