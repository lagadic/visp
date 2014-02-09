/****************************************************************************
 *
 * $Id: mbtTracking.cpp 3957 2012-11-07 15:22:30Z fnovotny $
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
 * Example of MBT KLT Tracking.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \example mbtKltTracking.cpp

  \brief Example of MBT KLT Tracking on an image sequence containing a cube.
*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpMath.h>
#include <visp/vpMbKltTracker.h>
#include <visp/vpVideoReader.h>
#include <visp/vpParseArgv.h>

#if defined (VISP_HAVE_OPENCV) && defined (VISP_HAVE_DISPLAY)


#define GETOPTARGS  "x:m:i:n:dchtfo"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &configFile, std::string &modelFile,
                std::string &initFile, bool &displayKltPoints, bool &click_allowed, bool &display,
                bool& cao3DModel, bool &useOgre);

void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Example of tracking based on the 3D model.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-x <config file>]\n\
  [-m <model name>] [-n <initialisation file base name>]\n\
  [-t] [-c] [-d] [-h] [-f]",
  name );

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
  -i <input image path>                                \n\
     Set image input path.\n\
     From this path read images \n\
     \"ViSP-images/mbt/cube/image%%04d.ppm\". These \n\
     images come from ViSP-images-x.y.z.tar.gz available \n\
     on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -x <config file>                                     \n\
     Set the config file (the xml file) to use.\n\
     The config file is used to specify the parameters of the tracker.\n\
\n\
  -m <model name>                                 \n\
     Specify the name of the file of the model\n\
     The model can either be a vrml model (.wrl) or a .cao file.\n\
\n\
  -f                                  \n\
     Do not use the vrml model, use the .cao one. These two models are \n\
     equivalent and comes from ViSP-images-x.y.z.tar.gz available on the ViSP\n\
     website. However, the .cao model allows to use the 3d model based tracker \n\
     without Coin.\n\
\n\
  -n <initialisation file base name>                                            \n\
     Base name of the initialisation file. The file will be 'base_name'.init .\n\
     This base name is also used for the optionnal picture specifying where to \n\
     click (a .ppm picture).\
\n\
  -t \n\
     Turn off the display of the the klt points. \n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -o\n\
     Use Ogre3D for visibility tests\n\
\n\
  -h \n\
     Print the help.\n\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}


bool getOptions(int argc, const char **argv, std::string &ipath, std::string &configFile, std::string &modelFile,
                std::string &initFile, bool &displayKltPoints, bool &click_allowed, bool &display,
                bool& cao3DModel, bool &useOgre)
{
  const char *optarg_;
  int   c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i': ipath = optarg_; break;
    case 'x': configFile = optarg_; break;
    case 'm': modelFile = optarg_; break;
    case 'n': initFile = optarg_; break;
    case 't': displayKltPoints = false; break;
    case 'f': cao3DModel = true; break;
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'o': useOgre = true; break;
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

int
main(int argc, const char ** argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string opt_configFile;
    std::string configFile;
    std::string opt_modelFile;
    std::string modelFile;
    std::string opt_initFile;
    std::string initFile;
    bool displayKltPoints = true;
    bool opt_click_allowed = true;
    bool opt_display = true;
    bool cao3DModel = false;
    bool useOgre = false;

    // Get the VISP_IMAGE_PATH environment variable value
    char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
    if (ptenv != NULL)
      env_ipath = ptenv;

    // Set the default input path
    if (! env_ipath.empty())
      ipath = env_ipath;


    // Read the command line options
    if (!getOptions(argc, argv, opt_ipath, opt_configFile, opt_modelFile, opt_initFile, displayKltPoints, opt_click_allowed, opt_display, cao3DModel, useOgre)) {
      return (-1);
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty() ){
      usage(argv[0], NULL);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
                << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;

      return (-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath + vpIoTools::path("/ViSP-images/mbt/cube/image%04d.pgm");
    else
      ipath = env_ipath + vpIoTools::path("/ViSP-images/mbt/cube/image%04d.pgm");

    if (!opt_configFile.empty())
      configFile = opt_configFile;
    else if (!opt_ipath.empty())
      configFile = opt_ipath + vpIoTools::path("/ViSP-images/mbt/cube.xml");
    else
      configFile = env_ipath + vpIoTools::path("/ViSP-images/mbt/cube.xml");

    if (!opt_modelFile.empty()){
      modelFile = opt_modelFile;
    }else{
      std::string modelFileCao = "/ViSP-images/mbt/cube.cao";
      std::string modelFileWrl = "/ViSP-images/mbt/cube.wrl";

      if(!opt_ipath.empty()){
        if(cao3DModel){
          modelFile = opt_ipath + vpIoTools::path(modelFileCao);
        }
        else{
#ifdef VISP_HAVE_COIN
          modelFile = opt_ipath + vpIoTools::path(modelFileWrl);
#else
          std::cerr << "Coin is not detected in ViSP. Use the .cao model instead." << std::endl;
          modelFile = opt_ipath + vpIoTools::path(modelFileCao);
#endif
        }
      }
      else{
        if(cao3DModel){
          modelFile = env_ipath + vpIoTools::path(modelFileCao);
        }
        else{
#ifdef VISP_HAVE_COIN
          modelFile = env_ipath + vpIoTools::path(modelFileWrl);
#else
          std::cerr << "Coin is not detected in ViSP. Use the .cao model instead." << std::endl;
          modelFile = env_ipath + vpIoTools::path(modelFileCao);
#endif
        }
      }
    }

    if (!opt_initFile.empty())
      initFile = opt_initFile;
    else if (!opt_ipath.empty())
      initFile = opt_ipath + vpIoTools::path("/ViSP-images/mbt/cube");
    else
      initFile = env_ipath + vpIoTools::path("/ViSP-images/mbt/cube");

    vpImage<unsigned char> I;
    vpVideoReader reader;

    reader.setFileName(ipath.c_str());
    try{
      reader.open(I);
    }catch(...){
      std::cout << "Cannot open sequence: " << ipath << std::endl;
      return -1;
    }

    reader.acquire(I);

    // initialise a  display
#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
#elif defined VISP_HAVE_D3D9
    vpDisplayD3D display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#else
    opt_display = false;
#endif
    if (opt_display)
    {
#if (defined VISP_HAVE_DISPLAY)
      display.init(I, 100, 100, "Test tracking") ;
#endif
      vpDisplay::display(I) ;
      vpDisplay::flush(I);
    }

    vpMbKltTracker tracker;
    vpHomogeneousMatrix cMo;

    // Load tracker config file (camera parameters and moving edge settings)
    vpCameraParameters cam;
#if defined (VISP_HAVE_XML2)
    // From the xml file
    tracker.loadConfigFile(configFile.c_str());
#else
    // By setting the parameters:
    cam.initPersProjWithoutDistortion(547, 542, 338, 234);

    vpKltOpencv klt;
    klt.setMaxFeatures(10000);
    klt.setWindowSize(5);
    klt.setQuality(0.01);
    klt.setMinDistance(5);
    klt.setHarrisFreeParameter(0.01);
    klt.setBlockSize(3);
    klt.setPyramidLevels(3);

    tracker.setCameraParameters(cam);
    tracker.setKltOpencv(klt);
    tracker.setAngleAppear( vpMath::rad(65) );
    tracker.setAngleDisappear( vpMath::rad(75) );
    tracker.setMaskBorder(5);

    // Specify the clipping to use
    tracker.setNearClippingDistance(0.01);
    tracker.setFarClippingDistance(0.90);
    tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
    //   tracker.setClipping(tracker.getClipping() | vpMbtPolygon::LEFT_CLIPPING | vpMbtPolygon::RIGHT_CLIPPING | vpMbtPolygon::UP_CLIPPING | vpMbtPolygon::DOWN_CLIPPING); // Equivalent to FOV_CLIPPING
#endif

    // Display the klt points
    tracker.setDisplayFeatures(displayKltPoints);

    // Tells if the tracker has to use Ogre3D for visibility tests
    tracker.setOgreVisibilityTest(useOgre);

    // Retrieve the camera parameters from the tracker
    tracker.getCameraParameters(cam);

    // Loop to position the cube
    if (opt_display && opt_click_allowed)
    {
      while(!vpDisplay::getClick(I,false)){
        vpDisplay::display(I);
        vpDisplay::displayCharString(I, 15, 10,
                                     "click after positioning the object",
                                     vpColor::red);
        vpDisplay::flush(I) ;
      }
    }

    // Load the 3D model (either a vrml file or a .cao file)
    tracker.loadModel(modelFile.c_str());

    // Initialise the tracker by clicking on the image
    // This function looks for
    //   - a ./cube/cube.init file that defines the 3d coordinates (in meter, in the object basis) of the points used for the initialisation
    //   - a ./cube/cube.ppm file to display where the user have to click (optionnal, set by the third parameter)
    if (opt_display && opt_click_allowed)
    {
      tracker.initClick(I, initFile.c_str(), true);
      tracker.getPose(cMo);
      // display the 3D model at the given pose
      tracker.display(I,cMo, cam, vpColor::red);
    }
    else
    {
      vpHomogeneousMatrix cMoi(0.02044769891, 0.1101505452, 0.5078963719, 2.063603907, 1.110231561, -0.4392789872);
      tracker.initFromPose(I, cMoi);
    }

    //track the model
    tracker.track(I);
    tracker.getPose(cMo);

    if (opt_display)
      vpDisplay::flush(I);

    // Uncomment if you want to compute the covariance matrix.
    // tracker.setCovarianceComputation(true); //Important if you want tracker.getCovarianceMatrix() to work.

    while (!reader.end())
    {
      // acquire a new image 
      reader.acquire(I);
      // display the image
      if (opt_display)
        vpDisplay::display(I);
      // track the object
      tracker.track(I);
      tracker.getPose(cMo);
      // display the 3D model  
      if (opt_display)
      {
        tracker.display(I, cMo, cam, vpColor::darkRed);
        // display the frame
        vpDisplay::displayFrame (I, cMo, cam, 0.05, vpColor::blue);
      }
      
      // Uncomment if you want to print the covariance matrix. 
      // Make sure tracker.setCovarianceComputation(true) has been called (uncomment below).
      // std::cout << tracker.getCovarianceMatrix() << std::endl << std::endl;

      vpDisplay::flush(I) ;
    }
    reader.close();

#if defined (VISP_HAVE_XML2)
    // Cleanup memory allocated by xml library used to parse the xml config file in vpMbKltTracker::loadConfigFile()
    vpXmlParser::cleanup();
#endif

#ifdef VISP_HAVE_COIN
    // Cleanup memory allocated by Coin library used to load a vrml model in vpMbKltTracker::loadModel()
    SoDB::finish();
#endif

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}

#else

int main()
{
  std::cout << "OpenCV and display are required." << std::endl;
  return 0;
  
}

#endif
