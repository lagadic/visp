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
 * Test for image conversions.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <stdlib.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTime.h>


/*!
  \example testConversion.cpp

  \brief Manipulation of image conversions.

*/

// List of allowed command line options
#define GETOPTARGS	"cdi:o:h"

void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, std::string user);

/*

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user)
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
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, std::string user)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i': ipath = optarg_; break;
    case 'o': opath = optarg_; break;
    case 'h': usage(argv[0], NULL, ipath, opath, user); return false; break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user);
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
    std::string opt_opath;
    std::string ipath;
    std::string opath;
    std::string filename;
    std::string username;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (! env_ipath.empty())
      ipath = env_ipath;

    // Set the default output path
#if defined(_WIN32)
    opt_opath = "C:/temp";
#else
    opt_opath = "/tmp";
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
    opath = vpIoTools::createFilePath(opath, username);

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
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");
    vpCTRACE << "Load " <<  filename << std::endl;
    vpImageIo::read(Ig, filename) ;
    // Create a color image from the grey
    vpImageConvert::convert(Ig, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color.ppm");
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(Ic, filename) ;

    //-------------------- .ppm -> .pgm
    vpTRACE("Convert a color image (.ppm) to a grey image (.pgm)");
    // Load a color image from the disk
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    // Create a grey image from the color
    vpImageConvert::convert(Ic, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey.pgm");
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(Ig, filename) ;

    //-------------------- YUV -> RGB
    unsigned char y=187, u=10, v=30;
    unsigned char r, g, b;

    // Convert a YUV pixel value to a RGB value
    vpImageConvert::YUVToRGB(y, u, v, r, g, b);
    vpTRACE("y(%d) u(%d) v(%d) = r(%d) g(%d) b(%d)", y, u, v, r, g, b);

#ifdef VISP_HAVE_OPENCV
#if VISP_HAVE_OPENCV_VERSION < 0x020408
    double t0 = vpTime::measureTimeMs();
    /////////////////////////
    // Convert a IplImage to a vpImage<vpRGBa>
    ////////////////////////
    IplImage* image = NULL; /*!< The image read / acquired */
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    vpCTRACE << "Reading the color image with opencv: "<< std::endl
             << filename << std::endl;
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(image, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cv.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    if(image!=NULL) cvReleaseImage( &image );
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(image, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cv.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ///////////////////////////
    // Convert a IplImage to a vpImage<unsigned char>
    ////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    vpCTRACE << "Reading the color image with opencv: "<< std::endl
             << filename << std::endl;
    if(image!=NULL) cvReleaseImage( &image );
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(image, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cv.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    if(image!=NULL) cvReleaseImage( &image );
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;

      return (-1);
    }
    vpImageConvert::convert(image, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cv.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> to a IplImage
    ////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    vpImageConvert::convert(Ic, image);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_color_cv.ppm");
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
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the grey image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ig, filename) ;
    vpImageConvert::convert(Ig, image);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_grey_cv.pgm");
    /* Save the the current image */

    vpCTRACE << "Write " << filename << std::endl;
    if((cvSaveImage(filename.c_str(), image)) == 0) {
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      if(image!=NULL) cvReleaseImage( &image );
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    if(image!=NULL) cvReleaseImage( &image );
    double t1 = vpTime::measureTimeMs();
    std::cout << "Conversion c interface : " << t1 - t0 << " ms" << std::endl;
#endif

    /* ------------------------------------------------------------------------ */
    /*                  conversion for the new c++ interface                    */
    /* ------------------------------------------------------------------------ */

#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    double t2 = vpTime::measureTimeMs();
    /////////////////////////
    // Convert a cv::Mat to a vpImage<vpRGBa>
    ////////////////////////
    cv::Mat imageMat;
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
    vpCTRACE << "Reading the color image with c++ interface of opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 1);// force to a three channel color image.
    if(imageMat.data == NULL){
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return -1;
    }
    vpImageConvert::convert(imageMat, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cvMat.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");
    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 0);// forced to grayscale.
    if(imageMat.data == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(imageMat, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cvMat.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ///////////////////////////
    // Convert a cv::Mat to a vpImage<unsigned char>
    ////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    vpCTRACE << "Reading the color image with opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 1);// force to a three channel color image.
    if(imageMat.data == NULL){
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return -1;
    }
    vpImageConvert::convert(imageMat, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cvMat.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 0);
    if(imageMat.data == NULL){
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(imageMat, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cvMat.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> to a cv::Mat
    ////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    vpImageConvert::convert(Ic, imageMat);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_color_cvMat.ppm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    if(!cv::imwrite(filename, imageMat)){
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////////
    // Convert a IplImage to a vpImage<unsigned char>
    ////////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the grey image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ig, filename);
    vpImageConvert::convert(Ig, imageMat);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_grey_cvMat.pgm");
    /* Save the the current image */

    vpCTRACE << "Write " << filename << std::endl;
    if(!cv::imwrite(filename, imageMat)){
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;
    double t3 = vpTime::measureTimeMs();
    std::cout << "Conversion c++ interface : " << t3 - t2 << " ms" << std::endl;
#endif
#endif

    ////////////////////////////////////
    // Split a vpImage<vpRGBa> to vpImage<unsigned char>
    ////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    vpImage<unsigned char> R,G,B,a;
    vpImageConvert::split(Ic, &R,NULL,&B);
    double begintime  = vpTime::measureTimeMs();
    for(int i=0; i<1000;i++){
      vpImageConvert::split(Ic, &R,NULL,&B);
    }
    double endtime = vpTime::measureTimeMs();

    std::cout<<"Time for 1000 split (ms): "<< endtime - begintime <<std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_RChannel.pgm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(R, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename =  vpIoTools::createFilePath(opath, "Klimt_BChannel.pgm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(B, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////
    // Merge 4 vpImage<unsigned char> (RGBa) to vpImage<vpRGBa>
    ////////////////////////////////////
    vpImageConvert::split(Ic, &R, &G, &B, &a);
    begintime  = vpTime::measureTimeMs();
    vpImage<vpRGBa> I_merge;
    for(int i=0; i<1000; i++){
      vpImageConvert::merge(&R, &G, &B, &a, I_merge);
    }
    endtime = vpTime::measureTimeMs();

    std::cout<<"Time for 1000 merge (ms): "<< endtime - begintime <<std::endl;

    filename =  vpIoTools::createFilePath(opath, "Klimt_merge.ppm");
    /* Save the the current image */
    vpImageIo::write(I_merge, filename) ;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> in RGB color space to a vpImage<vpRGBa> in HSV color
    ////////////////////////////////////
    unsigned int size = Ic.getWidth()*Ic.getHeight();
    unsigned int w = Ic.getWidth(), h = Ic.getHeight();
    unsigned char *hue = new unsigned char[size];
    unsigned char *saturation = new unsigned char[size];
    unsigned char *value = new unsigned char[size];

    vpImageConvert::RGBaToHSV((unsigned char *) Ic.bitmap, hue, saturation, value, size);
    vpImage<unsigned char> I_hue(hue, h, w);
    vpImage<unsigned char> I_saturation(saturation, h, w);
    vpImage<unsigned char> I_value(value, h, w);
    vpImage<vpRGBa> I_HSV;
    vpImageConvert::merge(&I_hue, &I_saturation, &I_value, NULL, I_HSV);

    filename =  vpIoTools::createFilePath(opath, "Klimt_HSV.ppm");
    /* Save the the current image */
    vpImageIo::write(I_HSV, filename);

    //Check the conversion RGBa <==> HSV
    double *hue2 = new double[size];
    double *saturation2 = new double[size];
    double *value2 = new double[size];
    vpImageConvert::RGBaToHSV((unsigned char *) Ic.bitmap, hue2, saturation2, value2, size);

    unsigned char *rgba = new unsigned char[size*4];
    vpImageConvert::HSVToRGBa(hue2, saturation2, value2, rgba, size);

    if(hue2 != NULL) {
      delete[] hue2;
      hue2 = NULL;
    }

    if(saturation2 != NULL) {
      delete[] saturation2;
      saturation2 = NULL;
    }

    if(value2 != NULL) {
      delete[] value2;
      value2 = NULL;
    }

    vpImage<vpRGBa> I_HSV2RGBa((vpRGBa *) rgba, h, w);
    filename =  vpIoTools::createFilePath(opath, "Klimt_HSV2RGBa.ppm");
    /* Save the the current image */
    vpImageIo::write(I_HSV2RGBa, filename);

    for(unsigned int i = 0; i < Ic.getHeight(); i++) {
      for(unsigned int j = 0; j < Ic.getWidth(); j++) {
        if(Ic[i][j].R != I_HSV2RGBa[i][j].R ||
           Ic[i][j].G != I_HSV2RGBa[i][j].G ||
           Ic[i][j].B != I_HSV2RGBa[i][j].B) {
          std::cerr << "Ic[i][j].R=" << static_cast<unsigned>(Ic[i][j].R)
              << " ; I_HSV2RGBa[i][j].R=" << static_cast<unsigned>(I_HSV2RGBa[i][j].R) << std::endl;
          std::cerr << "Ic[i][j].G=" << static_cast<unsigned>(Ic[i][j].G)
              << " ; I_HSV2RGBa[i][j].G=" << static_cast<unsigned>(I_HSV2RGBa[i][j].G) << std::endl;
          std::cerr << "Ic[i][j].B=" << static_cast<unsigned>(Ic[i][j].B)
              << " ; I_HSV2RGBa[i][j].B=" << static_cast<unsigned>(I_HSV2RGBa[i][j].B) << std::endl;
          throw vpException(vpException::fatalError, "Problem with conversion between RGB <==> HSV");
        }
      }
    }

    ////////////////////////////////////
    // Test construction of vpImage from an array with copyData==true
    ////////////////////////////////////
    unsigned char *rgba2 = new unsigned char[size*4];
    memset(rgba2, 127, size*4);
    vpImage<vpRGBa> I_copyData((vpRGBa *) rgba2, h, w, true);

    //Delete the array
    if(rgba2 != NULL) {
      delete[] rgba2;
      rgba2 = NULL;
    }

    filename =  vpIoTools::createFilePath(opath, "I_copyData.ppm");
    /* Save the the current image */
    vpImageIo::write(I_copyData, filename);

    if(I_copyData.getSize() > 0) {
      I_copyData[0][0].R = 10;
    }

    return 0;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
