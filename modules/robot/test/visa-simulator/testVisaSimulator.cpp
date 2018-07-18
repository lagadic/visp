/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Description:
 * Visa simulator adpator.
 *
 * Authors:
 * Andrey Kudryavtsev (Femto-ST)
 *
 *****************************************************************************/

/*! \example testVisaSimulator.cpp */

#include <iostream>

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/robot/vpVisaSocketAdaptor.h>

int main ()
{
  // init communication with simulator
  vpVisaSocketAdapter visa;
  visa.connect();

  std::vector<double> calibMatrix;
  visa.getCalibMatrix(calibMatrix);
  double px = calibMatrix[0];
  double py = calibMatrix[4];
  double u0 = calibMatrix[6];
  double v0 = calibMatrix[7];

  std::cout << "Focal distances (x,y) = (" << px << ", " << py << ")" << std::endl;
  std::cout << "Principal point = (" << u0 << ", " << v0 << ")" << std::endl;
  // image capture
  vpImage<unsigned char> I(v0*2, u0*2, 0);
#ifdef VISP_HAVE_X11
  vpDisplayX d(I, 100, 100, "-- current image --") ;
#elif VISP_HAVE_GDI
  vpDisplayGDI d(I, 100, 100, "-- current image --") ;
#elif VISP_HAVE_OPENCV
  vpDisplayOpenCV d(I, 100, 100, "-- current image --") ;
#endif

  // Camera parameters
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

  while(1)
  {
    double t = vpTime::measureTimeMs();

    I = visa.getImageViSP();
    vpDisplay::display(I);
    vpDisplay::displayCharString(I, 10, 10, "Mouse right click on the image to select feature points ...",vpColor::orange);
    vpMouseButton::vpMouseButtonType button;
    vpDisplay::getClick(I, button, false);
    vpDisplay::flush(I);
    if(button == vpMouseButton::button3) {
      break;
    }
    vpDisplay::flush(I);

    vpTime::wait(t, 40); // Loop time is set to 40 ms, ie 25 Hz
  }

  vpDisplay::display(I);
  vpDisplay::flush(I);

  // iterations: dot tracker
  vpDot2 blobs[4]; // detected blobs
  vpImagePoint blobsCOG; // blob gravity center
  double Z = 0.05; // depth of desired points

  std::ofstream myfile;

  std::cout << "                           " << std::endl;
  std::cout << "TAKE A DESIRED POSITION ..." << std::endl;
  std::cout << "                           " << std::endl;

  vpDisplay::display(I);
  vpDisplay::displayCharString(I, 10, 10, "Click on the dot to initialize the tracker", vpColor::darkGreen);
  vpDisplay::displayCharString(I, 30, 10, "Order: top-left,top-right,bottom-right,bottom-left", vpColor::darkGreen);

  vpDisplay::flush(I);

  vpFeaturePoint pd[4] ;
  std::string filePrefix = "coord_desired" + std::to_string(0) + ".txt";
  myfile.open (filePrefix);
  for ( int i = 0 ; i < 4 ; i++ )
  {
    blobs[i].initTracking(I) ;
    blobsCOG = blobs[i].getCog();

    vpDisplay::displayCross(I, blobsCOG, 10, vpColor::blue) ;
    vpDisplay::flush(I);
    double x = 0, y = 0;
    vpPixelMeterConversion::convertPoint(cam, blobsCOG, x, y) ;
    pd[i].set_xyZ(x, y, Z);
    std::cout << "pd_X[" << i << "] = " << pd[i].get_x() << std::endl;
    std::cout << "pd_Y[" << i << "] = " << pd[i].get_y() << std::endl;
    std::cout << "pd_Z[" << i << "] = " << pd[i].get_Z() << std::endl;
    myfile << blobs[i].getCog().get_i() << std::endl;
    myfile << blobs[i].getCog().get_j() << std::endl;
  }
  vpDisplay::close(I);
  myfile.close();
}
