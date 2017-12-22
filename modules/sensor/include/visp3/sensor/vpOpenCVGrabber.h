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
 * Cameras video capture using OpenCV library.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpOpenCVGrabber.h
  \brief class for cameras video capture using OpenCV library.
*/

#ifndef vpOpenCVGrabber_h
#define vpOpenCVGrabber_h

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408))

#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#include <opencv2/highgui/highgui.hpp>
#else
#include <highgui.h>
#endif

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpOpenCVGrabber

  \ingroup group_sensor_camera

  \brief Class for cameras video capture using OpenCV library.
  \deprecated This class is deprecated can only be used with OpenCV < 2.4.8.

  \note Instead of using this class, it is also possible to grab images using
OpenCV library by using directly OpenCV cv::VideoCapture class. The following
code corresponding to tutorial-grabber-opencv.cpp shows how to grab images in
a cv::Mat structure and then convert OpenCV images in ViSP images. \include
tutorial-grabber-opencv.cpp The one in grabOpencv.cpp gives an other example.

  The code below shows how to grab and
  display images using this class.
  \code
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/sensor/vpOpenCVGrabber.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408)
  try {
    vpImage<unsigned char> I;

    vpOpenCVGrabber g;
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;
    vpDisplayOpenCV d(I);

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
  \endcode
 */
class VISP_EXPORT vpOpenCVGrabber : public vpFrameGrabber
{
private:
  CvCapture *capture;
  int DeviceType;
  bool flip;

public:
  vpOpenCVGrabber();
  ~vpOpenCVGrabber();

  void open();
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);
  IplImage *acquire();

  void close();

  void getFramerate(double &framerate);
  void setFramerate(const double framerate);

  void setWidth(const unsigned int width);
  void setHeight(const unsigned int height);

  void setDeviceType(int type);

  void setFlip(bool flipType);
};

#endif
#endif
