/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test threading capabilities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \example testThreadedCapture.cpp

  \brief Test camera capture and display in separate threads.

*/

#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

#include <opencv2/highgui/highgui.hpp>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000)

bool image_available = false;
vpMutex mutex;
cv::Mat frame;

vpThread::Return captureFunction(vpThread::Args args)
{
  std::cout << "Start capture thread" << std::endl;
  cv::VideoCapture cap = *((cv::VideoCapture *) args);

//  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  if(!cap.isOpened()) { // check if we succeeded
    std::cout << "Unable to start capture" << std::endl;
    return 0;
  }

  double start_time = vpTime::measureTimeMs();
  while ((vpTime::measureTimeMs() - start_time) < 4000) {
    //std::cout << "capture in progress" << std::endl;
    mutex.lock();
    image_available = true;
    cap >> frame; // get a new frame from camera
    mutex.unlock();
  }
  mutex.lock();
  image_available = false;
  mutex.unlock();

  std::cout << "End of capture thread" << std::endl;
  return 0;
}

vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args
  std::cout << "Start display thread" << std::endl;
  vpImage<unsigned char> I;

  bool image_available_ = false;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *d = NULL;
#endif

  do {
    mutex.lock();
    image_available_ = image_available;
    mutex.unlock();

    // Initialize the display with the first available image
    if (! display_initialized_) {
      if (image_available_) {
        mutex.lock();
        vpImageConvert::convert(frame, I);
        mutex.unlock();
#if defined(VISP_HAVE_X11)
        d = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
        d = new vpDisplayGDI;
#endif
        d->init(I);
        display_initialized_ = true;
      }
    }
    else {
      mutex.lock();
      vpImageConvert::convert(frame, I);
      mutex.unlock();
      //std::cout << "Display image" << std::endl;
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    vpTime::wait(20);

  } while(image_available_ || (! display_initialized_));

  delete d;

  std::cout << "End of display thread" << std::endl;
  return 0;
}

int main() 
{
  int device = 0;
  cv::VideoCapture cap; // open the default camera

  cap.open(device);

  vpThread thread_capture;
  vpThread thread_display;

  thread_capture.create(captureFunction, (vpThread::Args)&cap);
  thread_display.create(displayFunction);

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();

  return 0;
}

#else
int main()
{
  std::cout << "You should install OpenCV to make this example working..." << std::endl;
}

#endif
