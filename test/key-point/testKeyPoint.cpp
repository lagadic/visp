/****************************************************************************
 *
 * $Id: testKeyPoint.cpp 5202 2015-01-24 09:29:06Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Test auto detection of dots.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <iostream>
#if ((defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_OPENCV))

#include <visp/vpKeyPoint.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpVideoReader.h>
#include <visp/vpIoTools.h>


/*!
  \example testKeyPoint.cpp

  \brief   Test matching keypoints.
*/
int main() {
  try {
    std::string env_ipath;
    //Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if(env_ipath.empty()) {
      std::cerr << "Please get the visp-images-data package path or set the VISP_INPUT_IMAGE_PATH "
          "environment variable value." << std::endl;
      return -1;
    }

    vpImage<unsigned char> Iref, Icur, Imatch;

    //Set the path location of the image sequence
    std::string dirname = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube");

    //Build the name of the image files
    std::string filenameRef = vpIoTools::createFilePath(dirname, "image0000.pgm");
    vpImageIo::read(Iref, filenameRef);
    std::string filenameCur = vpIoTools::createFilePath(dirname, "image%04d.pgm");

    //Init keypoints
    vpKeyPoint keypoints("ORB", "ORB");
    std::cout << "Build " << keypoints.buildReference(Iref) << " reference points." << std::endl;;


    vpVideoReader g;
    g.setFileName(filenameCur);
    g.open(Icur);
    g.acquire(Icur);

    Imatch.resize(Icur.getHeight(), 2*Icur.getWidth());
    Imatch.insert(Iref, vpImagePoint(0,0));

#if defined VISP_HAVE_X11
    vpDisplayX display(Imatch, 0, 0, "ORB keypoints matching");
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display(Imatch, 0, 0, "ORB keypoints matching");
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display(Imatch, 0, 0, "ORB keypoints matching");
#else
    std::cerr << "No display available." << std::endl;
    return -1;
#endif

    bool opt_click = false;
    vpMouseButton::vpMouseButtonType button;
    while(!g.end()) {
      double t = vpTime::measureTimeMs();
      g.acquire(Icur);
      Imatch.insert(Icur, vpImagePoint(0, Icur.getWidth()));

      vpDisplay::display(Imatch);

      //Match keypoints
      keypoints.matchPoint(Icur);
      //Display image with keypoints matched
      keypoints.displayMatching(Iref, Imatch);

      vpDisplay::flush(Imatch);
      t = vpTime::measureTimeMs() - t;

      //Click requested to process next image
      if(opt_click) {
        vpDisplay::getClick(Imatch, button, true);
        if(button == vpMouseButton::button3) {
          opt_click = false;
        }
      } else {
        //Use right click to enable/disable step by step tracking
        if(vpDisplay::getClick(Imatch, button, false)) {
          if (button == vpMouseButton::button3) {
            opt_click = true;
          }
          else if(button == vpMouseButton::button1) {
            break;
          }
        }
      }

      vpTime::wait(t, 30);
    }

  } catch(vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
#else
int main() {
#if ( !(defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)) )
  std::cerr << "You do not have X11, GTK or GDI display functionalities." << std::endl;
#else
  std::cerr << "You need OpenCV library." << std::endl;
#endif

  return -1;
}

#endif
