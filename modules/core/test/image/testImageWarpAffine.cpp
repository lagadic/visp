/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Test for vpImageTools::warpAffine() function.
 *
 * Author:
 * Vikas Thamizharasan
 *
 *****************************************************************************/
/*!
  \example testImageWarpAffine.cpp

  \brief Test the warpAffine function with different types of transformation matrix
*/

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageTools.h>

// List of allowed command line options
#define GETOPTARGS "cdip:h"


namespace {
  /*
    Print the program options.

    \param name : Program name.
    \param badparam : Bad parameter name.
    \param ipath: Input image path.
    \param ppath: Personal image path.
   */
  void usage(const char *name, const char *badparam, std::string ipath, std::string ppath)
  {
    fprintf(stdout, "\n\
  Test image warping.\n\
  \n\
  SYNOPSIS\n\
    %s [-i <input image path>] [-p <personal image path>] [-c] [-d]\n\
       [-h]\n                 \
  ", name);

    fprintf(stdout, "\n\
  OPTIONS:                                               Default\n\
    -i <input image path>                                %s\n\
       Set image input path.\n\
       From this path read \"ViSP-images/Klimt/Klimt.pgm\"\n\
       image.\n\
       Setting the VISP_INPUT_IMAGE_PATH environment\n\
       variable produces the same behaviour than using\n\
       this option.\n\
  \n\
    -p <personal image path>                               %s\n\
       Path to an image used to test image warping.\n\
       Example: -p /my_path_to/image.png\n\
  \n\
    -c                                   \n\
       Disable mouse click.\n\
  \n\
    -d                                   \n\
       Disable image display.\n\
  \n\
    -h\n\
       Print the help.\n\n",
      ipath.c_str(),
      ppath.c_str());

    if (badparam)
      fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
  }

  /*!
    Set the program options.

    \param argc : Command line number of parameters.
    \param argv : Array of command line parameters.
    \param ipath: Input image path.
    \param ppath: Personal image path.
    \param opt_display : Do not display if set.
    \param opt_click : Do not need click if set.
    \return false if the program has to be stopped, true otherwise.
  */
  bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath,
                  bool &opt_display, bool &opt_click)
  {
    const char *optarg_;
    int c;
    while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

      switch (c) {
      // case 'i': ipath = optarg_; break;
      case 'p': ppath = optarg_; break;
      case 'h': usage(argv[0], NULL, ipath, ppath); return false; break;

      case 'c': opt_click = false; break;
      case 'd': opt_display = false; break;

      default:
        usage(argv[0], optarg_, ipath, ppath); return false; break;
      }
    }

    if ((c == 1) || (c == -1)) {
      // standalone param or error
      usage(argv[0], NULL, ipath, ppath);
      std::cerr << "ERROR: " << std::endl;
      std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
      return false;
    }

    return true;
  }
}

int main(int argc, const char ** argv) {
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string opt_ppath;
    std::string ppath;
    std::string filename;
    bool opt_display = true;
    bool opt_click = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_display, opt_click) == false) {
      exit (EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;
    
    if (!opt_ppath.empty()) 
      ppath = opt_ppath;    

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (opt_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl
                  << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()){
      usage(argv[0], NULL, ipath, ppath);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
                << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl << std::endl;
      exit(EXIT_FAILURE);
    }

    std::cout << "Testing vpImageTools::warpAffine() ..." << std::endl;

    //
    // Test starts from here
    //
    //Initialize transformation matrix
    vpMatrix T(3,3);

    // Scale (Sx,Sy), Rotation (angle), Translation (tx,ty)
    float scale_x, scale_y, angle;
    signed int a_0, b_0;

    // Test Scale along Y
    // scale_x = 1; scale_y = 1.5; angle = 0; a_0 = b_0 = 0;

    // Test Rotation
    scale_x = 1; scale_y = 1; angle = 60; a_0 = b_0 = 0;
    
    // Test Translation
    // scale_x = 1; scale_y = 1; angle = 0; a_0 = 10; b_0 = 20;

    // Test Rotation, Scale and Translation
    // scale_x = 0.6; scale_y = 1.3; angle = 45; a_0 = 100; b_0 = 77;

    // Make Tansformation Matrix
    T[0][0] = scale_x * cos(vpMath::rad(angle));  T[0][1] = scale_x * sin(vpMath::rad(angle));  T[0][2] = 0;
    T[1][0] = -scale_y * sin(vpMath::rad(angle)); T[1][1] = scale_y * cos(vpMath::rad(angle));  T[1][2] = 0;
    T[2][0] = a_0;                                T[2][1] = b_0;                                T[2][2] = 1;

    //Load Image (default Klimt.pgm)
    vpImage<vpRGBa> I_image, I_warped_image;

    if (!opt_ppath.empty())    
        filename = ppath;
    else
        filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    vpImageIo::read(I_image, filename);

    vpImageTools::warpAffine(I_image, T, I_warped_image);

    std::cout << "Final Image size() : " <<  I_warped_image.getRows() << " x " << I_warped_image.getCols() << std::endl;    

#if defined (VISP_HAVE_X11)
    vpDisplayX *d1 = new vpDisplayX, *d2 = new vpDisplayX;
#elif defined (VISP_HAVE_OPENCV)
    vpDisplayOpenCV *d1 = new vpDisplayOpenCV, *d2 = new vpDisplayOpenCV;
#elif defined (VISP_HAVE_GTK)
    vpDisplayGTK *d1 = new vpDisplayGTK, *d2 = new vpDisplayGTK;
#elif defined (VISP_HAVE_GDI)
    vpDisplayGDI *d1 = new vpDisplayGDI, *d2 = new vpDisplayGDI;
#elif defined (VISP_HAVE_D3D9)
    vpDisplayD3D *d1 = new vpDisplayD3D, *d2 = new vpDisplayD3D;
#else
    std::cerr << "No display available!" << std::endl;
    opt_display = false;
#endif

    if (opt_display) {
#if defined (VISP_HAVE_X11) || defined (VISP_HAVE_OPENCV) || defined (VISP_HAVE_GTK) || defined (VISP_HAVE_GDI) || defined (VISP_HAVE_D3D9)
      d1->init(I_image, 0, 0, "Original image");
      d2->init(I_warped_image, (int) I_image.getWidth()+50, 0, "Warped image");
#endif

      vpDisplay::display(I_image);
      vpDisplay::display(I_warped_image);
      vpDisplay::displayText(I_warped_image, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I_image);
      vpDisplay::flush(I_warped_image);
      if (opt_click) {
        vpDisplay::getClick(I_warped_image);
      }
    }

#if defined (VISP_HAVE_X11) || defined (VISP_HAVE_OPENCV) || defined (VISP_HAVE_GTK) || defined (VISP_HAVE_GDI) || defined (VISP_HAVE_D3D9)
    delete d1;
    delete d2;
#endif
  } catch(vpException &e) {
    std::cerr << "\nCatch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\nTest Successful" << std::endl;
  return EXIT_SUCCESS;
}
