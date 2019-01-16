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
 * Implementation of a simple augmented reality application using the vpAROgre
 * class.
 *
 * Authors:
 * Bertrand Delabarre
 *
 *****************************************************************************/

/*!
  \example AROgreBasic.cpp
  Very simple example of augmented reality based on Ogre3D.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

//#if defined(VISP_HAVE_OGRE) && (defined(VISP_HAVE_OPENCV) ||
// defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_GTK)
//|| (defined(VISP_HAVE_X11) && ! defined(APPLE)))
#if defined(VISP_HAVE_OGRE) &&                                                                                         \
    (defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_GTK) ||       \
     (defined(VISP_HAVE_X11) && !(defined(__APPLE__) && defined(__MACH__))))

//#if defined(VISP_HAVE_X11) && ! defined(APPLE)
#if defined(VISP_HAVE_X11) && !(defined(__APPLE__) && defined(__MACH__))
// produce an error on OSX: ‘typedef int Cursor’
// /usr/X11R6/include/X11/X.h:108: error: ‘Cursor’ has a previous
// declaration as ‘typedef XID Cursor’. That's why it should not be
// used on APPLE platforms
#include <visp3/gui/vpDisplayX.h>
#endif
#include <visp3/ar/vpAROgre.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpPose.h>

// List of allowed command line options
#define GETOPTARGS "ci:p:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.

*/
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath)
{
  fprintf(stdout, "\n\
Test augmented reality using the vpAROgre class.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-c] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read images \n\
     \"mire-2/image.%%04d.pgm\". These \n\
     images come from ViSP-images-x.y.z.tar.gz available \n\
     on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
 -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     The following image file formats PNM (PGM P5, PPM P6)\n\
     are supported. The format is selected by analysing \n\
     the filename extension.\n\
     Example : \"/Temp/ViSP-images/cube/image.%%04d.pgm\"\n\
     %%04d is for the image numbering.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -h\n\
     Print the help.\n", ipath.c_str(), ppath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param click_allowed : Mouse click activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, bool &click_allowed)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c':
      click_allowed = false;
      break;
    case 'i':
      ipath = optarg;
      break;
    case 'p':
      ppath = optarg;
      break;
    case 'h':
      usage(argv[0], NULL, ipath, ppath);
      return false;
      break;

    default:
      usage(argv[0], optarg, ipath, ppath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  This function computes a pose from four black points.
  Here to keep dimensions coherency you will need those four dots to be
  situated at (-7,6,0),(7,6,0),(7,-6,0),(-7,-6,0) (unit = cm) in your real
  world
*/
void computeInitialPose(vpCameraParameters *mcam, vpImage<unsigned char> &I, vpPose *mPose, vpDot2 *md,
                        vpImagePoint *mcog, vpHomogeneousMatrix *cMo, vpPoint *mP, const bool &opt_click_allowed)
{
  // ---------------------------------------------------
  //    Code inspired from ViSP example of camera pose
  // ----------------------------------------------------
  bool opt_display = true;

//#if defined(VISP_HAVE_X11) && ! defined(APPLE)
#if defined(VISP_HAVE_X11) && !(defined(__APPLE__) && defined(__MACH__))
  // produce an error on OSX: ‘typedef int Cursor’
  // /usr/X11R6/include/X11/X.h:108: error: ‘Cursor’ has a previous
  // declaration as ‘typedef XID Cursor’. That's why it should not be
  // used on APPLE platforms
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV display;
#elif defined VISP_HAVE_D3D9
  vpDisplayD3D display;
#endif

  for (unsigned int i = 0; i < 4; i++) {
    if (opt_display) {
      md[i].setGraphics(true);
    } else {
      md[i].setGraphics(false);
    }
  }

  if (opt_display) {
    try {
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100, "Preliminary Pose Calculation");
      // display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      // Flush the display
      vpDisplay::flush(I);

    } catch (...) {
      vpERROR_TRACE("Error while displaying the image");
      return;
    }
  }

  std::cout << "*************************************************************"
               "***********************"
            << std::endl;
  std::cout << "*************************** Preliminary Pose Calculation "
               "***************************"
            << std::endl;
  std::cout << "******************************  Click on the 4 dots  "
               "*******************************"
            << std::endl;
  std::cout << "********Dot1 : (-x,-y,0), Dot2 : (x,-y,0), Dot3 : (x,y,0), "
               "Dot4 : (-x,y,0)**********"
            << std::endl;
  std::cout << "*************************************************************"
               "***********************"
            << std::endl;

  try {
    vpImagePoint ip[4];
    if (!opt_click_allowed) {
      ip[0].set_i(265);
      ip[0].set_j(93);
      ip[1].set_i(248);
      ip[1].set_j(242);
      ip[2].set_i(166);
      ip[2].set_j(215);
      ip[3].set_i(178);
      ip[3].set_j(85);
    }

    for (unsigned int i = 0; i < 4; i++) {
      // by using setGraphics, we request to see the edges of the dot
      // in red on the screen.
      // It uses the overlay image plane.
      // The default of this setting is that it is time consumming

      md[i].setGraphics(true);
      md[i].setGrayLevelPrecision(0.7);
      md[i].setSizePrecision(0.5);

      for (unsigned int j = 0; j < i; j++)
        md[j].display(I);

      // flush the display buffer
      vpDisplay::flush(I);
      try {
        if (opt_click_allowed) {
          md[i].initTracking(I);
          // std::cout << "click " << i << " " << md[i] << std::endl;
        } else {
          md[i].initTracking(I, ip[i]);
        }
      } catch (...) {
      }

      mcog[i] = md[i].getCog();
      // an expcetion is thrown by the track method if
      //  - dot is lost
      //  - the number of pixel is too small
      //  - too many pixels are detected (this is usual when a "big"
      //  specularity
      //    occurs. The threshold can be modified using the
      //    setNbMaxPoint(int) method
      if (opt_display) {
        md[i].display(I);
        // flush the display buffer
        vpDisplay::flush(I);
      }
    }
  } catch (const vpException &e) {
    vpERROR_TRACE("Error while tracking dots");
    vpCTRACE << e;
    return;
  }

  if (opt_display) {
    // display a red cross (size 10) in the image at the dot center
    // of gravity location
    //
    // WARNING
    // in the vpDisplay class member's when pixel coordinates
    // are considered the first element is the row index and the second
    // is the column index:
    //   vpDisplay::displayCross(Image, row index, column index, size, color)
    //   therefore u and v are inverted wrt to the vpDot specification
    // Alternatively, to avoid this problem another set of member have
    // been defined in the vpDisplay class.
    // If the method name is postfixe with _uv the specification is :
    //   vpDisplay::displayCross_uv(Image, column index, row index, size,
    //   color)

    for (unsigned int i = 0; i < 4; i++)
      vpDisplay::displayCross(I, mcog[i], 10, vpColor::red);

    // flush the X11 buffer
    vpDisplay::flush(I);
  }

  // --------------------------------------------------------
  //             Now we will compute the pose
  // --------------------------------------------------------

  //  the list of point is cleared (if that's not done before)
  mPose->clearPoint();

  // we set the 3D points coordinates (in meter !) in the object/world frame
  double l = 0.06;
  double L = 0.07;
  mP[0].setWorldCoordinates(-L, -l, 0); // (X,Y,Z)
  mP[1].setWorldCoordinates(L, -l, 0);
  mP[2].setWorldCoordinates(L, l, 0);
  mP[3].setWorldCoordinates(-L, l, 0);

  // pixel-> meter conversion
  for (unsigned int i = 0; i < 4; i++) {
    // u[i]. v[i] are expressed in pixel
    // conversion in meter is achieved using
    // x = (u-u0)/px
    // y = (v-v0)/py
    // where px, py, u0, v0 are the intrinsic camera parameters
    double x = 0, y = 0;
    vpPixelMeterConversion::convertPoint(*mcam, mcog[i], x, y);
    mP[i].set_x(x);
    mP[i].set_y(y);
  }

  // The pose structure is build, we put in the point list the set of point
  // here both 2D and 3D world coordinates are known
  for (unsigned int i = 0; i < 4; i++) {
    mPose->addPoint(mP[i]); // and added to the pose computation point list
  }

  // compute the initial pose using Dementhon method followed by a non linear
  // minimisation method

  // Pose by Lagrange it provides an initialization of the pose
  mPose->computePose(vpPose::LAGRANGE, *cMo);
  // the pose is now refined using the virtual visual servoing approach
  // Warning: cMo needs to be initialized otherwise it may  diverge
  mPose->computePose(vpPose::VIRTUAL_VS, *cMo);

  // Display breifly just to have a glimpse a the ViSP pose
  //	while(cpt<500){
  if (opt_display) {
    // Display the computed pose
    mPose->display(I, *cMo, *mcam, 0.05, vpColor::red);
    vpDisplay::flush(I);
    vpTime::wait(1000);
  }
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string opt_ppath;
    std::string dirname;
    std::string filename;
    bool opt_click_allowed = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_click_allowed) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty() && opt_ppath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty() && opt_ppath.empty()) {
      usage(argv[0], NULL, ipath, opt_ppath);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << "  Use -p <personal image path> option if you want to " << std::endl
                << "  use personal images." << std::endl
                << std::endl;

      exit(-1);
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    //	  vpImage<unsigned char> I ;

    //	  unsigned iter = 0;
    std::ostringstream s;
    //	  char cfilename[FILENAME_MAX];

    if (opt_ppath.empty()) {
      // Set the path location of the image sequence
      dirname = vpIoTools::createFilePath(ipath, "mire-2");

      // Build the name of the image file

      s.setf(std::ios::right, std::ios::adjustfield);
      s << "image.%04d.pgm";
      filename = vpIoTools::createFilePath(dirname, s.str());
    } else {
      filename = opt_ppath;
    }

    // We will read a sequence of images
    vpVideoReader grabber;
    grabber.setFirstFrameIndex(1);
    grabber.setFileName(filename.c_str());
    // Grey level image associated to a display in the initial pose
    // computation
    vpImage<unsigned char> Idisplay;
    // Grey level image to track points
    vpImage<unsigned char> I;
    // RGBa image to get background
    vpImage<vpRGBa> IC;
    // Matrix representing camera parameters
    vpHomogeneousMatrix cMo;

    // Variables used for pose computation purposes
    vpPose mPose;
    vpDot2 md[4];
    vpImagePoint mcog[4];
    vpPoint mP[4];

    // CameraParameters we got from calibration
    // Keep u0 and v0 as center of the screen
    vpCameraParameters mcam;

    // Read the PGM image named "filename" on the disk, and put the
    // bitmap into the image structure I.  I is initialized to the
    // correct size
    //
    // exception readPGM may throw various exception if, for example,
    // the file does not exist, or if the memory cannot be allocated
    try {
      vpCTRACE << "Load: " << filename << std::endl;
      grabber.open(Idisplay);
      grabber.acquire(Idisplay);
      vpCameraParameters mcamTmp(592, 570, grabber.getWidth() / 2, grabber.getHeight() / 2);
      // Compute the initial pose of the camera
      computeInitialPose(&mcamTmp, Idisplay, &mPose, md, mcog, &cMo, mP, opt_click_allowed);
      // Close the framegrabber
      grabber.close();

      // Associate the grabber to the RGBa image
      grabber.open(IC);
      mcam.init(mcamTmp);
    } catch (...) {
      // an exception is thrown if an exception from readPGM has been caught
      // here this will result in the end of the program
      // Note that another error message has been printed from readPGM
      // to give more information about the error
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filename << std::endl;
      std::cerr << "  Check your -i " << ipath << " option " << std::endl
                << "  or VISP_INPUT_IMAGE_PATH environment variable." << std::endl;
      exit(-1);
    }

    // Create a vpRAOgre object with color background
    vpAROgre ogre(mcam, grabber.getWidth(), grabber.getHeight());
    // Initialize it
    ogre.init(IC);
    ogre.load("Robot", "robot.mesh");
    ogre.setScale("Robot", 0.001f, 0.001f, 0.001f);
    ogre.setRotation("Robot", vpRotationMatrix(vpRxyzVector(M_PI / 2, -M_PI / 2, 0)));

    // Add an optional point light source
    Ogre::Light *light = ogre.getSceneManager()->createLight();
    light->setDiffuseColour(1, 1, 1);  // scaled RGB values
    light->setSpecularColour(1, 1, 1); // scaled RGB values
    light->setPosition(-5, -5, 10);
    light->setType(Ogre::Light::LT_POINT);

    // Rendering loop
    while (ogre.continueRendering() && !grabber.end()) {
      // Acquire a frame
      grabber.acquire(IC);

      // Convert it to a grey level image for tracking purpose
      vpImageConvert::convert(IC, I);

      // kill the point list
      mPose.clearPoint();

      // track the dot
      for (int i = 0; i < 4; i++) {
        // track the point
        md[i].track(I, mcog[i]);
        md[i].setGrayLevelPrecision(0.90);
        // pixel->meter conversion
        {
          double x = 0, y = 0;
          vpPixelMeterConversion::convertPoint(mcam, mcog[i], x, y);
          mP[i].set_x(x);
          mP[i].set_y(y);
        }

        // and added to the pose computation point list
        mPose.addPoint(mP[i]);
      }
      // the pose structure has been updated

      // the pose is now updated using the virtual visual servoing approach
      // Dementhon or lagrange is no longuer necessary, pose at the
      // previous iteration is sufficient
      mPose.computePose(vpPose::VIRTUAL_VS, cMo);

      // Display with ogre
      ogre.display(IC, cMo);

      // Wait so that the video does not go too fast
      vpTime::wait(15);
    }
    // Close the grabber
    grabber.close();
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  } catch (Ogre::Exception &e) {
    std::cout << "Catch an Ogre exception: " << e.getDescription() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cout << "Catch an exception " << std::endl;
    return EXIT_FAILURE;
  }
}
#else // VISP_HAVE_OGRE && VISP_HAVE_DISPLAY
int main()
{
#if (!(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)))
  std::cout << "You do not have X11, or GTK, or GDI (Graphical Device Interface) functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
#else
  std::cout << "You do not have Ogre functionalities" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install Ogre3D, configure again ViSP using cmake and build again this example" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
