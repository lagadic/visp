/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Test for image conversions.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <stdlib.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>
#include <visp/vpTime.h>
/*!
  \example testConversion.cpp

  \brief Manipulation of image conversions.

*/

// List of allowed command line options
#define GETOPTARGS	"i:o:h"

/*

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.

 */
void usage(const char *name, const char *badparam, std::string ipath, 
	   std::string opath, std::string user)
{
  fprintf(stdout, "\n\
Test image conversions.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>]\n\
     [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/Klimt/Klimt.pgm\"\n\
     and \"ViSP-images/Klimt/Klimt.ppm\" images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     Klimt_grey.pgm and Klimt_color.ppm output images\n\
     are written.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), opath.c_str(), user.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv,
		std::string &ipath, std::string &opath, std::string user)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'i': ipath = optarg; break;
    case 'o': opath = optarg; break;
    case 'h': usage(argv[0], NULL, ipath, opath, user); return false; break;

    default:
      usage(argv[0], optarg, ipath, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

int
main(int argc, const char ** argv)
{

  std::string env_ipath;
  std::string opt_ipath;
  std::string opt_opath;
  std::string ipath;
  std::string opath;
  std::string filename;
  std::string username;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Set the default output path
#ifdef UNIX
  opt_opath = "/tmp";
#elif WIN32
  opt_opath = "C:\\temp";
#endif

  // Get the user login name
  vpIoTools::getUserName(username);

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_opath, username) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;
  if (!opt_opath.empty())
    opath = opt_opath;

  // Append to the output path string, the login name of the user
  opath += vpIoTools::path("/") + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(opath) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(opath);
    }
    catch (...) {
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl
	   << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << opath << std::endl;
      std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
      exit(-1);
    }
  }

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
    usage(argv[0], NULL, ipath, opt_opath, username);
    std::cerr << std::endl
	 << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << std::endl
	 << "  environment variable to specify the location of the " << std::endl
	 << "  image path where test images are located." << std::endl << std::endl;
    exit(-1);
  }

  // 
  // Here starts really the test
  // 

  vpImage<unsigned char> Ig ; // Grey image
  vpImage<vpRGBa> Ic ; // Color image

  //-------------------- .pgm -> .ppm
  vpTRACE("Convert a grey image (.pgm) to a color image (.ppm)");
  // Load a grey image from the disk
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");
  vpCTRACE << "Load " <<  filename << std::endl;
  vpImageIo::readPGM(Ig, filename) ;
  // Create a color image from the grey
  vpImageConvert::convert(Ig, Ic);
  filename = opath +  vpIoTools::path("/Klimt_color.ppm");
  vpCTRACE << "Write " << filename << std::endl;
  vpImageIo::writePPM(Ic, filename) ;

  //-------------------- .ppm -> .pgm
  vpTRACE("Convert a color image (.ppm) to a grey image (.pgm)");
  // Load a color image from the disk
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");
  vpCTRACE << "Load " << filename << std::endl;
  vpImageIo::readPPM(Ic, filename) ;
  // Create a grey image from the color
  vpImageConvert::convert(Ic, Ig);
  filename = opath +  vpIoTools::path("/Klimt_grey.pgm");
  vpCTRACE << "Write " << filename << std::endl;
  vpImageIo::writePPM(Ig, filename) ;

  //-------------------- YUV -> RGB
  unsigned char y=187, u=10, v=30;
  unsigned char r, g, b;

  // Convert a YUV pixel value to a RGB value
  vpImageConvert::YUVToRGB(y, u, v, r, g, b);
  vpTRACE("y(%d) u(%d) v(%d) = r(%d) g(%d) b(%d)", y, u, v, r, g, b);

#ifdef VISP_HAVE_OPENCV
/////////////////////////
// Convert a IplImage to a vpImage<vpRGBa>
////////////////////////
  IplImage* image = NULL; /*!< The image read / acquired */
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");

  /* Read the color image */

  vpCTRACE << "Reading the color image with opencv: "<< std::endl
           << filename << std::endl;
  if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR)) == NULL) {
    vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
    return (-1);
  }
  vpImageConvert::convert(image, Ic);
  filename = opath +  vpIoTools::path("/Klimt_color_cv.ppm");
  /* Save the the current image */
    vpImageIo::writePPM(Ic, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;
    
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");

  /* Read the pgm image */

  vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
           << filename << std::endl;
  if(image!=NULL) cvReleaseImage( &image );
  if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == NULL) {
    vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
    return (-1);
  }
  vpImageConvert::convert(image, Ic);
  filename = opath +  vpIoTools::path("/Klimt_grey_cv.ppm");
  /* Save the the current image */
    vpImageIo::writePPM(Ic, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;
    
///////////////////////////
// Convert a IplImage to a vpImage<unsigned char>
////////////////////////////
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");

  /* Read the color image */

  vpCTRACE << "Reading the color image with opencv: "<< std::endl
           << filename << std::endl;
  if(image!=NULL) cvReleaseImage( &image );
  if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR)) == NULL) {
    vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
    return (-1);
  }
  vpImageConvert::convert(image, Ig);
  filename = opath +  vpIoTools::path("/Klimt_color_cv.pgm");
  /* Save the the current image */
    vpImageIo::writePGM(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;
    
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");

  /* Read the pgm image */

  vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
           << filename << std::endl;
  if(image!=NULL) cvReleaseImage( &image );
  if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == NULL) {
    vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
    
    return (-1);
  }
  vpImageConvert::convert(image, Ig);
  filename = opath +  vpIoTools::path("/Klimt_grey_cv.pgm");
  /* Save the the current image */
  vpImageIo::writePGM(Ig, filename) ;

  vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

////////////////////////////////////
// Convert a vpImage<vpRGBa> to a IplImage
////////////////////////////////////
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");

  /* Read the color image */

// Load a color image from the disk
  vpCTRACE << "Load " << filename << std::endl;
  vpImageIo::readPPM(Ic, filename) ;
  vpImageConvert::convert(Ic, image);
  filename = opath +  vpIoTools::path("/Klimt_ipl_color_cv.ppm");
  /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    if((cvSaveImage(filename.c_str(), image)) == 0) {
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      if(image!=NULL) cvReleaseImage( &image );
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

////////////////////////////////////////
// Convert a IplImage to a vpImage<unsigned char>
////////////////////////////////////////
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");

/* Read the grey image */

// Load a color image from the disk
  vpCTRACE << "Load " << filename << std::endl;
  vpImageIo::readPGM(Ig, filename) ;
  vpImageConvert::convert(Ig, image);
  filename = opath +  vpIoTools::path("/Klimt_ipl_grey_cv.pgm");
  /* Save the the current image */

    vpCTRACE << "Write " << filename << std::endl;
    if((cvSaveImage(filename.c_str(), image)) == 0) {
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      if(image!=NULL) cvReleaseImage( &image );
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

  if(image!=NULL) cvReleaseImage( &image );

#endif
  
  ////////////////////////////////////
// Split a vpImage<vpRGBa> to vpImage<unsigned char> 
  ////////////////////////////////////
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");

  /* Read the color image */

// Load a color image from the disk
  vpCTRACE << "Load " << filename << std::endl;
  vpImageIo::readPPM(Ic, filename) ;
  vpImage<unsigned char> R,G,B,a;
  vpImageConvert::split(Ic, &R,NULL,&B); 
  double begintime  = vpTime::measureTimeMs();
  for(int i=0; i<1000;i++){
    vpImageConvert::split(Ic, &R,NULL,&B);
  }
  double endtime = vpTime::measureTimeMs();
  
  std::cout<<"Time for 1000 split (ms): "<< endtime - begintime <<std::endl;
      
  filename = opath +  vpIoTools::path("/Klimt_RChannel.pgm");
  /* Save the the current image */
  vpCTRACE << "Write " << filename << std::endl;
  vpImageIo::writePGM(R, filename) ;
  vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

  filename = opath +  vpIoTools::path("/Klimt_BChannel.pgm");
  /* Save the the current image */
  vpCTRACE << "Write " << filename << std::endl;
  vpImageIo::writePGM(B, filename) ;
  vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
