/****************************************************************************
 *
 * $Id: testKeyPoint-2.cpp 5202 2015-01-24 09:29:06Z fspindle $
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
 * Test keypoint matching and pose estimation.
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
#include <visp/vpMbEdgeTracker.h>


/*!
  \example testKeyPoint.cpp

  \brief   Test keypoint matching and pose estimation.
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

    vpImage<unsigned char> I;

    //Set the path location of the image sequence
    std::string dirname = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube");

    //Build the name of the image files
    std::string filenameRef = vpIoTools::createFilePath(dirname, "image0000.pgm");
    vpImageIo::read(I, filenameRef);
    std::string filenameCur = vpIoTools::createFilePath(dirname, "image%04d.pgm");


#if defined VISP_HAVE_X11
    vpDisplayX display(I, 0, 0, "ORB keypoints matching");
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display(Imatch, 0, 0, "ORB keypoints matching");
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display(Imatch, 0, 0, "ORB keypoints matching");
#else
    std::cerr << "No display available." << std::endl;
    return -1;
#endif


    vpMbEdgeTracker tracker;
    //Load config for tracker
    std::string tracker_config_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.xml");
    tracker.loadConfigFile(tracker_config_file);
    tracker.setAngleAppear(vpMath::rad(89));
    tracker.setAngleDisappear(vpMath::rad(89));

    vpCameraParameters cam;
    tracker.getCameraParameters(cam);

    //Load CAO model
    std::string cao_model_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.cao");
    tracker.loadModel(cao_model_file);

    //Initialize the pose
    std::string init_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.init");
    tracker.initClick(I, init_file);

    //Get the init pose
    vpHomogeneousMatrix cMo;
    tracker.getPose(cMo);


    //Init keypoints
    vpKeyPoint keypoints("ORB", "ORB");

    //Detect keypoints on the current image
    std::vector<cv::KeyPoint> trainKeyPoints;
    double elapsedTime;
    keypoints.detect(I, trainKeyPoints, elapsedTime);

    //Keep only keypoints on the cube
    std::vector<vpPolygon> polygons;
    std::vector<std::vector<vpPoint> > roisPt;
    std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
    polygons = pair.first;
    roisPt = pair.second;

    //Compute the 3D coordinates
    std::vector<cv::Point3f> points3f;
    vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);

    //Build the reference keypoints
    keypoints.buildReference(I, trainKeyPoints, points3f);


    //Init reader for getting the input image sequence
    vpVideoReader g;
    g.setFileName(filenameCur);
    g.open(I);
    g.acquire(I);

    bool opt_click = false;
    double error;
    vpMouseButton::vpMouseButtonType button;
    while(!g.end()) {
      g.acquire(I);

      vpDisplay::display(I);

      //Match keypoints and estimate the pose
      if(keypoints.matchPoint(I, cam, cMo, error, elapsedTime)) {
        tracker.setPose(I, cMo);
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      }

      vpDisplay::flush(I);

      //Click requested to process next image
      if(opt_click) {
        vpDisplay::getClick(I, button, true);
        if(button == vpMouseButton::button3) {
          opt_click = false;
        }
      } else {
        //Use right click to enable/disable step by step tracking
        if(vpDisplay::getClick(I, button, false)) {
          if (button == vpMouseButton::button3) {
            opt_click = true;
          }
          else if(button == vpMouseButton::button1) {
            break;
          }
        }
      }
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
