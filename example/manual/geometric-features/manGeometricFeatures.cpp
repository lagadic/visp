/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Geometric features example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGeometricFeatures.cpp

  \brief Geometric features projection example.

*/
/*!
  \example manGeometricFeatures.cpp

  \brief Geometric features projection example.

*/

#include <visp3/core/vpDebug.h>
#include <visp3/io/vpImageIo.h>
// For 2D image
#include <visp3/core/vpImage.h>
// Video device interface
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

// For frame transformation and projection
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>

// Needed geometric features
#include <visp3/core/vpCircle.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpSphere.h>

#include <iostream>

int main()
{
  try {
    std::cout << "ViSP geometric features display example" << std::endl;
    unsigned int height = 288;
    unsigned int width = 384;
    vpImage<unsigned char> I(height, width);
    I = 255; // I is a white image

    // create a display window
#if defined(VISP_HAVE_X11)
    vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display;
#else
  std::cout << "Please install X11, GDI, OpenCV or GTK to see the result of this example" << std::endl;
#endif


#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)
    // initialize a display attached to image I
    display.init(I, 100, 100, "ViSP geometric features display");
#endif

    // camera parameters to digitalize the image plane
    vpCameraParameters cam(600, 600, width / 2, height / 2); // px,py,u0,v0

    // pose of the camera with reference to the scene
    vpTranslationVector t(0, 0, 1);
    vpRxyzVector rxyz(-M_PI / 4, 0, 0);
    vpRotationMatrix R(rxyz);
    vpHomogeneousMatrix cMo(t, R);

    // scene building, geometric features definition
    vpPoint point;
    point.setWorldCoordinates(0, 0, 0); // (X0=0,Y0=0,Z0=0)
    vpLine line;
    line.setWorldCoordinates(1, 1, 0, 0, 0, 0, 1, 0); // planes:(X+Y=0)&(Z=0)
    vpCylinder cylinder;
    cylinder.setWorldCoordinates(1, -1, 0, 0, 0, 0,
                                 0.1); // alpha=1,beta=-1,gamma=0,
    // X0=0,Y0=0,Z0=0,R=0.1
    vpCircle circle;
    circle.setWorldCoordinates(0, 0, 1, 0, 0, 0,
                               0.1); // plane:(Z=0),X0=0,Y0=0,Z=0,R=0.1
    vpSphere sphere;
    sphere.setWorldCoordinates(0, 0, 0, 0.1); // X0=0,Y0=0,Z0=0,R=0.1

    // change frame to be the camera frame and project features in the image
    // plane
    point.project(cMo);
    line.project(cMo);
    cylinder.project(cMo);
    circle.project(cMo);
    sphere.project(cMo);

    // display the scene
    vpDisplay::display(I); // display I
    // draw the projections of the 3D geometric features in the image plane.
    point.display(I, cam, vpColor::black);   // draw a black cross over I
    line.display(I, cam, vpColor::blue);     // draw a blue line over I
    cylinder.display(I, cam, vpColor::red);  // draw two red lines over I
    circle.display(I, cam, vpColor::orange); // draw an orange ellipse over I
    sphere.display(I, cam, vpColor::black);  // draw a black ellipse over I

    vpDisplay::flush(I); // flush the display buffer
    vpDisplay::displayText(I, 10, 10, "Click in the display to exit", vpColor::red);
    vpDisplay::getClick(I); // wait for a click in the display to exit

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)
    // save the drawing
    vpImage<vpRGBa> Ic;
    vpDisplay::getImage(I, Ic);
    std::cout << "ViSP creates \"./geometricFeatures.ppm\" image" << std::endl;
    vpImageIo::write(Ic, "./geometricFeatures.ppm");
#endif
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
