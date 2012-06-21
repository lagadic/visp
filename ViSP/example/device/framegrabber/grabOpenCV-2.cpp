/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Acquire images using OpenCV cv::VideoCapture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

/*!
  \example grabOpenCV-2.cpp

  \brief Example of framegrabbing using OpenCV cv::VideoCapture class.

*/

#include <iostream>

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)

#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpOpenCVGrabber.h>


// usage: binary <device name>
// device name: 0 is the default to dial with the first camera, 
//              1 to dial with a second camera attached to the computer
int main(int argc, char** argv)
{
  int device = 0;
  if (argc > 1)
    device = atoi(argv[1]);

  std::cout << "Use device: " << device << std::endl;
  cv::VideoCapture cap(device); // open the default camera
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  if(!cap.isOpened())  // check if we succeeded
    return -1;
  cv::Mat frame;
  cap >> frame; // get a new frame from camera

  IplImage iplimage = frame;
  std::cout << "Image size: " << iplimage.width << " " 
            << iplimage.height << std::endl;

  //vpImage<vpRGBa> I; // for color images
  vpImage<unsigned char> I; // for gray images
  vpImageConvert::convert(&iplimage, I);
  vpDisplayOpenCV d(I);

  for(;;) {
    cap >> frame; // get a new frame from camera
    iplimage = frame;

    // Convert the image in ViSP format and display it 
    vpImageConvert::convert(&iplimage, I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) // a click to exit
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}

#else
int main()
{
  std::cout << "OpenCV is not available..." << std::endl;
}

#endif
