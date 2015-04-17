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

#include <iostream>

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp/vpKeyPoint.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>
#include <visp/vpIoTools.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"cdi:h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test key points matching.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg_);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  \example testKeyPoint-2.cpp

  \brief   Test keypoint matching and pose estimation.
*/
int main(int argc, const char ** argv) {
  try {
    std::string env_ipath;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit (-1);
    }

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
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#else
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      display.init(I, 0, 0, "ORB keypoints matching and pose estimation");
    }

    vpCameraParameters cam;
    vpMbEdgeTracker tracker;
    //Load config for tracker
    std::string tracker_config_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.xml");

    bool usexml = false;
#ifdef VISP_HAVE_XML2
    tracker.loadConfigFile(tracker_config_file);
    tracker.getCameraParameters(cam);

    usexml = true;
#endif
    if (! usexml) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(547.7367575, 542.0744058, 338.7036994, 234.5083345);
      tracker.setCameraParameters(cam);
      tracker.setNearClippingDistance(0.01);
      tracker.setFarClippingDistance(100.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
    }

    tracker.setAngleAppear(vpMath::rad(89));
    tracker.setAngleDisappear(vpMath::rad(89));

    //Load CAO model
    std::string cao_model_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.cao");
    tracker.loadModel(cao_model_file);

    //Initialize the pose
    std::string init_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.init");
    if (opt_display && opt_click_allowed) {
      tracker.initClick(I, init_file);
    }
    else
    {
      vpHomogeneousMatrix cMoi(0.02044769891, 0.1101505452, 0.5078963719, 2.063603907, 1.110231561, -0.4392789872);
      tracker.initFromPose(I, cMoi);
    }

    //Get the init pose
    vpHomogeneousMatrix cMo;
    tracker.getPose(cMo);

    //Init keypoints
    vpKeyPoint keypoints("ORB", "ORB", "BruteForce-Hamming");
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
    //Bug when using LSH index with FLANN and OpenCV 2.3.1.
    //see http://code.opencv.org/issues/1741 (Bug #1741)
    keypoints.setMatcher("FlannBased");
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
      keypoints.setDetectorParameter("ORB", "nLevels", 1);
#endif
#endif


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

#if defined VISP_HAVE_X11
    vpDisplayX display2;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display2;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display2;
#else
    vpDisplayOpenCV display2;
#endif

    vpImage<unsigned char> IMatching;

    keypoints.createImageMatching(I, IMatching);

    if (opt_display) {
      display2.init(IMatching, 0, I.getHeight() + 80, "IMatching");
    }

    bool opt_click = false;
    double error;
    vpMouseButton::vpMouseButtonType button;
    while((opt_display && !g.end()) || (!opt_display && g.getFrameIndex() < 30)) {
      g.acquire(I);

      if(opt_display) {
        vpDisplay::display(I);

        //Display image matching
        keypoints.insertImageMatching(I, IMatching);

        vpDisplay::display(IMatching);
      }

      //Match keypoints and estimate the pose
      if(keypoints.matchPoint(I, cam, cMo, error, elapsedTime)) {
        tracker.setCameraParameters(cam);
        tracker.setPose(I, cMo);

        if(opt_display) {
          tracker.display(I, cMo, cam, vpColor::red, 2);
          vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);

          std::vector<vpImagePoint> ransacInliers = keypoints.getRansacInliers();
          std::vector<vpImagePoint> ransacOutliers = keypoints.getRansacOutliers();

          for(std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
            vpDisplay::displayCircle(I, *it, 4, vpColor::green);
            vpImagePoint imPt(*it);
            imPt.set_u(imPt.get_u() + I.getWidth());
            vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::green);
          }

          for(std::vector<vpImagePoint>::const_iterator it = ransacOutliers.begin(); it != ransacOutliers.end(); ++it) {
            vpDisplay::displayCircle(I, *it, 4, vpColor::red);
            vpImagePoint imPt(*it);
            imPt.set_u(imPt.get_u() + I.getWidth());
            vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::red);
          }

          keypoints.displayMatching(I, IMatching);

          //Display model in the image
          vpCameraParameters cam2;
          cam2.initPersProjWithoutDistortion(cam.get_px(), cam.get_py(), cam.get_u0() + I.getWidth(), cam.get_v0());
          tracker.setCameraParameters(cam2);
          tracker.setPose(IMatching, cMo);
          tracker.display(IMatching, cMo, cam2, vpColor::red, 2);
          vpDisplay::displayFrame(IMatching, cMo, cam2, 0.025, vpColor::none, 3);
        }
      }

      if(opt_display) {
        vpDisplay::flush(I);
        vpDisplay::flush(IMatching);
      }

      if (opt_click_allowed && opt_display) {
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
    }

  } catch(vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
#else
int main() {
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
