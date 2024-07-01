/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \example testConversion.cpp

  \brief Manipulation of image conversions.
*/

#include <iomanip>
#include <stdlib.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#if defined(VISP_HAVE_YARP)
#include <yarp/sig/ImageFile.h>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// List of allowed command line options
#define GETOPTARGS "cdi:o:n:h"

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \param nbiter : Iteration number.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user, int nbiter)
{
  fprintf(stdout, "\n\
Test image conversions.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>] [-n <nb benchmark iterations>]\n\
     [-h]\n\
",
name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"Klimt/Klimt.pgm\"\n\
     and \"Klimt/Klimt.ppm\" images.\n\
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
  -n <nb benchmark iterations>                               %d\n\
     Set the number of benchmark iterations.\n\
\n\
  -h\n\
     Print the help.\n\n",
          ipath.c_str(), opath.c_str(), user.c_str(), nbiter);

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
  \param nbIterations : Number of benchmark iterations.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, const std::string &user,
                int &nbIterations)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'o':
      opath = optarg_;
      break;
    case 'n':
      nbIterations = atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], nullptr, ipath, opath, user, nbIterations);
      return false;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user, nbIterations);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, opath, user, nbIterations);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string opt_opath;
    std::string ipath;
    std::string opath;
    std::string filename;
    std::string username;
    int nbIterations = 1;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
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
    if (getOptions(argc, argv, opt_ipath, opt_opath, username, nbIterations) == false) {
      return EXIT_FAILURE;
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
        usage(argv[0], nullptr, ipath, opt_opath, username, nbIterations);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << opath << std::endl;
        std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
        return EXIT_FAILURE;
      }
    }

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path coming from the command line option
    if (opt_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
          << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
          << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], nullptr, ipath, opt_opath, username, nbIterations);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << std::endl;
      return EXIT_FAILURE;
    }

    //
    // Here starts really the test
    //

    vpImage<unsigned char> Ig; // Grey image
    vpImage<vpRGBa> Ic;        // Color image

    //-------------------- .pgm -> .ppm
    std::cout << "** Convert a grey image (.pgm) to a color image (.ppm)" << std::endl;
    // Load a grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    std::cout << "   Load " << filename << std::endl;
    vpImageIo::read(Ig, filename);
    // Create a color image from the grey
    vpImageConvert::convert(Ig, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(Ic, filename);

    //-------------------- .ppm -> .pgm
    std::cout << "** Convert a color image (.ppm) to a grey image (.pgm)" << std::endl;
    // Load a color image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    std::cout << "   Load " << filename << std::endl;
    vpImageIo::read(Ic, filename);
    // Create a grey image from the color
    vpImageConvert::convert(Ic, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey.pgm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(Ig, filename);

    //-------------------- YUV -> RGB
    std::cout << "** Convert YUV pixel value to a RGB value" << std::endl;
    unsigned char y = 187, u = 10, v = 30;
    unsigned char r, g, b;

    // Convert a YUV pixel value to a RGB value
    vpImageConvert::YUVToRGB(y, u, v, r, g, b);
    std::cout << "   y(" << (int)y << ") u(" << (int)u << ") v(" << (int)v << ") = r(" << (int)r << ") g(" << (int)g
      << ") b(" << (int)b << ")" << std::endl;

    vpChrono chrono;

    /* ------------------------------------------------------------------------ */
    /*                  conversion for the new c++ interface                    */
    /* ------------------------------------------------------------------------ */

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC)
    chrono.start();
    /////////////////////////
    // Convert a cv::Mat to a vpImage<vpRGBa>
    ////////////////////////
    std::cout << "** Convert a cv::Mat to a vpImage<vpRGBa>" << std::endl;
    cv::Mat imageMat;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    std::cout << "   Reading the color image with c++ interface of opencv: " << filename << std::endl;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    int flags = cv::IMREAD_COLOR;
#else
    int flags = CV_LOAD_IMAGE_COLOR;
#endif
    imageMat = cv::imread(filename, flags); // Force to a three channel BGR color image.
    if (imageMat.data == nullptr) {
      std::cout << "   Cannot read image: " << filename << std::endl;
      return EXIT_FAILURE;
    }
    vpImageConvert::convert(imageMat, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cvMat.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(Ic, filename);

    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    /* Read the pgm image */

    std::cout << "   Reading the grayscale image with opencv: " << filename << std::endl;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    flags = cv::IMREAD_GRAYSCALE;
#else
    flags = CV_LOAD_IMAGE_GRAYSCALE;
#endif
    imageMat = cv::imread(filename, flags); // Forced to grayscale.
    if (imageMat.data == nullptr) {
      std::cout << "   Cannot read image: " << filename << std::endl;
      return EXIT_FAILURE;
    }
    vpImageConvert::convert(imageMat, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cvMat.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(Ic, filename);

    ///////////////////////////
    // Convert a cv::Mat to a vpImage<unsigned char>
    ////////////////////////////
    std::cout << "** Convert a cv::Mat to a vpImage<nsigned char>" << std::endl;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");

    /* Read the color image */

    std::cout << "   Reading the color image with opencv: " << filename << std::endl;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    flags = cv::IMREAD_COLOR;
#else
    flags = CV_LOAD_IMAGE_COLOR;
#endif
    imageMat = cv::imread(filename, flags); // Force to a three channel BGR color image.
    if (imageMat.data == nullptr) {
      std::cout << "   Cannot read image: " << filename << std::endl;
      return EXIT_FAILURE;
    }
    vpImageConvert::convert(imageMat, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cvMat.pgm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(Ig, filename);

    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");

    /* Read the pgm image */

    std::cout << "   Reading the greyscale image with opencv: " << filename << std::endl;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    flags = cv::IMREAD_GRAYSCALE;
#else
    flags = CV_LOAD_IMAGE_GRAYSCALE;
#endif
    imageMat = cv::imread(filename, flags);
    if (imageMat.data == nullptr) {
      std::cout << "   Cannot read image: " << filename << std::endl;
      return EXIT_FAILURE;
    }
    vpImageConvert::convert(imageMat, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cvMat.pgm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(Ig, filename);

    std::cout << "   Convert result in " << filename << std::endl;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> to a cv::Mat
    ////////////////////////////////////
    std::cout << "** Convert a vpImage<vpRGBa> to a cv::Mat" << std::endl;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    std::cout << "   Load " << filename << std::endl;
    vpImageIo::read(Ic, filename);
    vpImageConvert::convert(Ic, imageMat);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_color_cvMat.ppm");
    /* Save the the current image */
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    if (!cv::imwrite(filename, imageMat)) {
      std::cout << "   Cannot write image: " << filename << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "   Convert result in " << filename << std::endl;

    ////////////////////////////////////////
    // Convert a vpImage<unsigned char> to a cv::Mat
    ////////////////////////////////////////
    std::cout << "** Convert a vpImage<unsigned char> to a cv::Mat" << std::endl;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");

    /* Read the grey image */

    // Load a color image from the disk
    std::cout << "   Load " << filename << std::endl;
    vpImageIo::read(Ig, filename);
    vpImageConvert::convert(Ig, imageMat);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_grey_cvMat.pgm");
    /* Save the the current image */

    std::cout << "   Resulting image saved in: " << filename << std::endl;
    if (!cv::imwrite(filename, imageMat)) {
      std::cout << "   Cannot write image: " << filename << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "   Convert result in " << filename << std::endl;
    chrono.stop();
    std::cout << "== Conversion c++ interface : " << chrono.getDurationMs() << " ms" << std::endl;
#endif

    ////////////////////////////////////
    // Split a vpImage<vpRGBa> to vpImage<unsigned char>
    ////////////////////////////////////
    std::cout << "** Split a vpImage<vpRGBa> to vpImage<unsigned char>" << std::endl;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    std::cout << "   Load " << filename << std::endl;
    vpImageIo::read(Ic, filename);
    vpImage<unsigned char> R, G, B, a;
    vpImageConvert::split(Ic, &R, nullptr, &B);
    chrono.start();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      vpImageConvert::split(Ic, &R, nullptr, &B);
    }
    chrono.stop();

    std::cout << "   Time for " << nbIterations << " split (ms): " << chrono.getDurationMs() << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_RChannel.pgm");
    /* Save the the current image */
    std::cout << "   Save Klimt R channel: " << filename << std::endl;
    vpImageIo::write(R, filename);

    filename = vpIoTools::createFilePath(opath, "Klimt_BChannel.pgm");
    /* Save the the current image */
    std::cout << "   Save Klimt B channel: " << filename << std::endl;
    vpImageIo::write(B, filename);

    ////////////////////////////////////
    // Merge 4 vpImage<unsigned char> (RGBa) to vpImage<vpRGBa>
    ////////////////////////////////////
    std::cout << "** Merge 4 vpImage<unsigned char> (RGBa) to vpImage<vpRGBa>" << std::endl;
    vpImageConvert::split(Ic, &R, &G, &B, &a);
    chrono.start();
    vpImage<vpRGBa> I_merge;
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      vpImageConvert::merge(&R, &G, &B, &a, I_merge);
    }
    chrono.stop();

    std::cout << "   Time for 1000 merge (ms): " << chrono.getDurationMs() << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_merge.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(I_merge, filename);

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> in RGB color space to a vpImage<vpRGBa> in
    // HSV color
    ////////////////////////////////////
    std::cout << "** Convert a vpImage<vpRGBa> in RGB color space to a  vpImage<vpRGBa> in HSV color" << std::endl;
    unsigned int size = Ic.getSize();
    unsigned int w = Ic.getWidth(), h = Ic.getHeight();
    // Check the conversion RGBa <==> HSV(unsigned char)
    std::vector<unsigned char> hue(size);
    std::vector<unsigned char> saturation(size);
    std::vector<unsigned char> value(size);

    vpImageConvert::RGBaToHSV((unsigned char *)Ic.bitmap, &hue.front(), &saturation.front(), &value.front(), size);
    vpImage<unsigned char> I_hue(&hue.front(), h, w);
    vpImage<unsigned char> I_saturation(&saturation.front(), h, w);
    vpImage<unsigned char> I_value(&value.front(), h, w);
    vpImage<vpRGBa> I_HSV;
    vpImageConvert::merge(&I_hue, &I_saturation, &I_value, nullptr, I_HSV);

    filename = vpIoTools::createFilePath(opath, "Klimt_HSV.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(I_HSV, filename);

    vpImage<vpRGBa> Ic_from_hsv(Ic.getHeight(), Ic.getWidth());
    vpImageConvert::HSVToRGBa(&hue.front(), &saturation.front(), &value.front(), reinterpret_cast<unsigned char *>(Ic_from_hsv.bitmap), size);
    for (unsigned int i = 0; i < Ic.getHeight(); i++) {
      for (unsigned int j = 0; j < Ic.getWidth(); j++) {
        int precision = 10.; // Due to cast to unsigned char
        if ((!vpMath::equal(static_cast<unsigned>(Ic[i][j].R), static_cast<unsigned>(Ic_from_hsv[i][j].R), precision))
            || (!vpMath::equal(static_cast<unsigned>(Ic[i][j].G), static_cast<unsigned>(Ic_from_hsv[i][j].G), precision))
            || (!vpMath::equal(static_cast<unsigned>(Ic[i][j].B), static_cast<unsigned>(Ic_from_hsv[i][j].B), precision))) {
          std::cerr << "Ic[i][j].R=" << static_cast<unsigned>(Ic[i][j].R)
            << " ; Ic_from_hsv[i][j].R=" << static_cast<unsigned>(Ic_from_hsv[i][j].R) << " precision: " << precision << std::endl;
          std::cerr << "Ic[i][j].G=" << static_cast<unsigned>(Ic[i][j].G)
            << " ; Ic_from_hsv[i][j].G=" << static_cast<unsigned>(Ic_from_hsv[i][j].G) << " precision: " << precision << std::endl;
          std::cerr << "Ic[i][j].B=" << static_cast<unsigned>(Ic[i][j].B)
            << " ; Ic_from_hsv[i][j].B=" << static_cast<unsigned>(Ic_from_hsv[i][j].B) << " precision: " << precision << std::endl;
          throw vpException(vpException::fatalError, "Problem with conversion between RGB <==> HSV(unsigned char)");
        }
      }
    }
    // Check the conversion RGBa <==> HSV(double)
    std::vector<double> hue2(size);
    std::vector<double> saturation2(size);
    std::vector<double> value2(size);
    vpImageConvert::RGBaToHSV((unsigned char *)Ic.bitmap, &hue2.front(), &saturation2.front(), &value2.front(), size);

    std::vector<unsigned char> rgba(size * 4);
    vpImageConvert::HSVToRGBa(&hue2.front(), &saturation2.front(), &value2.front(), &rgba.front(), size);

    vpImage<vpRGBa> I_HSV2RGBa(reinterpret_cast<vpRGBa *>(&rgba.front()), h, w);
    filename = vpIoTools::createFilePath(opath, "Klimt_HSV2RGBa.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(I_HSV2RGBa, filename);

    for (unsigned int i = 0; i < Ic.getHeight(); i++) {
      for (unsigned int j = 0; j < Ic.getWidth(); j++) {
        if (Ic[i][j].R != I_HSV2RGBa[i][j].R || Ic[i][j].G != I_HSV2RGBa[i][j].G || Ic[i][j].B != I_HSV2RGBa[i][j].B) {
          std::cerr << "Ic[i][j].R=" << static_cast<unsigned>(Ic[i][j].R)
            << " ; I_HSV2RGBa[i][j].R=" << static_cast<unsigned>(I_HSV2RGBa[i][j].R) << std::endl;
          std::cerr << "Ic[i][j].G=" << static_cast<unsigned>(Ic[i][j].G)
            << " ; I_HSV2RGBa[i][j].G=" << static_cast<unsigned>(I_HSV2RGBa[i][j].G) << std::endl;
          std::cerr << "Ic[i][j].B=" << static_cast<unsigned>(Ic[i][j].B)
            << " ; I_HSV2RGBa[i][j].B=" << static_cast<unsigned>(I_HSV2RGBa[i][j].B) << std::endl;
          throw vpException(vpException::fatalError, "Problem with conversion between RGB <==> HSV(double)");
        }
      }
    }

    ////////////////////////////////////
    // Test construction of a vpImage from an array with copyData==true
    ////////////////////////////////////
    std::cout << "** Construction of a vpImage from an array with copyData==true" << std::endl;
    std::vector<unsigned char> rgba2(size * 4);
    std::fill(rgba2.begin(), rgba2.end(), 127);
    vpImage<vpRGBa> I_copyData(reinterpret_cast<vpRGBa *>(&rgba2.front()), h, w, true);

    filename = vpIoTools::createFilePath(opath, "I_copyData.ppm");
    std::cout << "   Resulting image saved in: " << filename << std::endl;
    vpImageIo::write(I_copyData, filename);

    if (I_copyData.getSize() > 0) {
      I_copyData[0][0].R = 10;
    }

    // Test color conversion
    {
      std::cout << "** Test color conversion" << std::endl;
      // RGBa to Grayscale
      vpImage<vpRGBa> I_color;
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
      vpImageIo::read(I_color, filename);

      // RGB to Grayscale conversion
      std::vector<unsigned char> rgb_array(I_color.getSize() * 3);
      vpImageConvert::RGBaToRGB((unsigned char *)I_color.bitmap, &rgb_array.front(), I_color.getSize());

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC)
      // BGR cv::Mat to Grayscale
      std::cout << "\n   BGR cv::Mat to Grayscale" << std::endl;
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
      cv::Mat colorMat = cv::imread(filename);
      std::cout << "   colorMat=" << colorMat.cols << "x" << colorMat.rows << std::endl;

      // Test RGB to Grayscale + Flip
      std::cout << "\n   RGB to Grayscale + Flip" << std::endl;
      std::vector<unsigned char> rgb2gray_flip_array_sse(I_color.getSize());
      vpImageConvert::RGBToGrey(&rgb_array.front(), &rgb2gray_flip_array_sse.front(), I_color.getWidth(),
                                I_color.getHeight(), true);
      vpImage<unsigned char> I_rgb2gray_flip_sse(&rgb2gray_flip_array_sse.front(), I_color.getHeight(),
                                                 I_color.getWidth());

      filename = vpIoTools::createFilePath(opath, "I_rgb2gray_flip_sse.pgm");
      std::cout << "   Resulting image saved in: " << filename << std::endl;
      vpImageIo::write(I_rgb2gray_flip_sse, filename);

      // Test BGR to Grayscale + Flip
      std::cout << "\n   Conversion BGR to Grayscale + Flip" << std::endl;
      std::vector<unsigned char> bgr2gray_flip_array_sse(I_color.getSize());
      vpImage<unsigned char> I_bgr2gray_flip_sse(&bgr2gray_flip_array_sse.front(), I_color.getHeight(),
                                                 I_color.getWidth());
      vpImageConvert::convert(colorMat, I_bgr2gray_flip_sse, true);

      filename = vpIoTools::createFilePath(opath, "I_bgr2gray_flip_sse.pgm");
      std::cout << "   Resulting image saved in: " << filename << std::endl;
      vpImageIo::write(I_bgr2gray_flip_sse, filename);

      // Test RGB to Grayscale + Flip + Crop
      std::cout << "\n   RGB to Grayscale + Flip + Crop" << std::endl;
      cv::Rect rect_roi(11, 17, 347, 449);
      cv::Mat colorMat_crop = colorMat(rect_roi);
      cv::Mat colorMat_crop_continuous = colorMat(rect_roi).clone();
      std::cout << "   colorMat_crop: " << colorMat_crop.cols << "x" << colorMat_crop.rows << " is continuous? "
        << colorMat_crop.isContinuous() << std::endl;
      std::cout << "   colorMat_crop_continuous: " << colorMat_crop_continuous.cols << "x" << colorMat_crop_continuous.rows
        << " is continuous? " << colorMat_crop_continuous.isContinuous() << std::endl;

      vpImage<vpRGBa> I_color_crop((unsigned int)(rect_roi.height - rect_roi.y),
                                   (unsigned int)(rect_roi.width - rect_roi.x));
      for (unsigned int i = (unsigned int)rect_roi.y; i < (unsigned int)rect_roi.height; i++) {
        for (unsigned int j = (unsigned int)rect_roi.x; j < (unsigned int)rect_roi.width; j++) {
          I_color_crop[(unsigned int)((int)i - rect_roi.y)][(unsigned int)((int)j - rect_roi.x)] = I_color[i][j];
        }
      }
      filename = vpIoTools::createFilePath(opath, "I_color_crop.ppm");
      std::cout << "   Resulting image saved in: " << filename << std::endl;
      vpImageIo::write(I_color_crop, filename);

      std::vector<unsigned char> rgb_array_crop(I_color_crop.getSize() * 3);
      vpImageConvert::RGBaToRGB((unsigned char *)I_color_crop.bitmap, &rgb_array_crop.front(), I_color_crop.getSize());

      std::vector<unsigned char> rgb2gray_flip_crop_array_sse(I_color_crop.getSize());
      vpImageConvert::RGBToGrey(&rgb_array_crop.front(), &rgb2gray_flip_crop_array_sse.front(), I_color_crop.getWidth(),
                                I_color_crop.getHeight(), true);
      vpImage<unsigned char> I_rgb2gray_flip_crop_sse(&rgb2gray_flip_crop_array_sse.front(), I_color_crop.getHeight(),
                                                      I_color_crop.getWidth());

      filename = vpIoTools::createFilePath(opath, "I_rgb2gray_flip_crop_sse.pgm");
      std::cout << "   Resulting image saved in: " << filename << std::endl;
      vpImageIo::write(I_rgb2gray_flip_crop_sse, filename);

      // Test BGR to Grayscale + Flip + Crop
      std::cout << "\n   BGR to Grayscale + Flip + Crop" << std::endl;
      vpImage<unsigned char> I_bgr2gray_flip_crop_sse(I_color_crop.getHeight(), I_color_crop.getWidth());
      vpImageConvert::convert(colorMat_crop_continuous, I_bgr2gray_flip_crop_sse, true);

      filename = vpIoTools::createFilePath(opath, "I_bgr2gray_flip_crop_sse.pgm");
      std::cout << "   Resulting image saved in: " << filename << std::endl;
      vpImageIo::write(I_bgr2gray_flip_crop_sse, filename);

      // Test BGR to Grayscale + Flip + Crop + No continuous Mat
      std::cout << "\n   BGR to Grayscale + Flip + Crop + No continuous Mat" << std::endl;
      vpImage<unsigned char> I_bgr2gray_flip_crop_no_continuous_sse(I_color_crop.getHeight(), I_color_crop.getWidth());
      vpImageConvert::convert(colorMat_crop, I_bgr2gray_flip_crop_no_continuous_sse, true);

      filename = vpIoTools::createFilePath(opath, "I_bgr2gray_flip_crop_no_continuous_sse.pgm");
      std::cout << "   Resulting image saved in: " << filename << std::endl;
      vpImageIo::write(I_bgr2gray_flip_crop_no_continuous_sse, filename);
#endif
      std::cout << "   Test succeed" << std::endl;
    }

#if defined(VISP_HAVE_YARP)
    /////////////////////////
    // Convert a ViSP to Yarp uchar image
    ////////////////////////
    std::cout << "** Test ViSP to Yarp image conversion by copy" << std::endl;
    {
      bool convert_by_copy = true;
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
      std::cout << "   Reading the gray image with ViSP: " << filename << std::endl;
      vpImage<unsigned char> I;
      // Read an image on a disk
      vpImageIo::read(I, filename);

      yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
      // Convert the vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>
      vpImageConvert::convert(I, Iyarp, convert_by_copy);
      // Write the image
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_copy.pgm");
      std::cout << "   Converted Yarp image saved in: " << filename << std::endl;
      yarp::sig::file::write(*Iyarp, filename, yarp::sig::file::FORMAT_PGM);

      std::cout << "   Reading the gray image with Yarp: " << filename << std::endl;
      yarp::sig::ImageOf< yarp::sig::PixelMono > *IIyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
      yarp::sig::file::read(*IIyarp, filename, yarp::sig::file::FORMAT_PGM);
      vpImage<unsigned char> II;
      vpImageConvert::convert(IIyarp, II, convert_by_copy);
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_copy_visp.pgm");
      std::cout << "   Converted image in ViSP saved in: " << filename << std::endl;
      vpImageIo::write(II, filename);
      if (I != II) {
        std::cout << "   Yarp gray conversion test failed" << std::endl;
        return EXIT_FAILURE;
      }
      std::cout << std::endl;
    }
    {
      bool convert_by_copy = true;
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
      std::cout << "   Reading the color image with ViSP: " << filename << std::endl;
      vpImage<vpRGBa> I;
      // Read an image on a disk
      vpImageIo::read(I, filename);

      yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
      // Convert the vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgba\>
      vpImageConvert::convert(I, Iyarp, convert_by_copy);
      // Write the image
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_copy.ppm");
      std::cout << "   Converted image saved in: " << filename << std::endl;
      yarp::sig::file::write(*Iyarp, filename, yarp::sig::file::FORMAT_PPM);

      std::cout << "   Reading the color image with Yarp: " << filename << std::endl;
      yarp::sig::ImageOf< yarp::sig::PixelRgba > *IIyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
      yarp::sig::file::read(*IIyarp, filename, yarp::sig::file::FORMAT_PPM);
      vpImage<vpRGBa> II;
      vpImageConvert::convert(IIyarp, II, convert_by_copy);
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_copy_visp.ppm");
      std::cout << "   Converted image in ViSP saved in: " << filename << std::endl;
      vpImageIo::write(II, filename);
      if (I != II) {
        std::cout << "   Yarp color conversion test failed" << std::endl;
        return EXIT_FAILURE;
      }
      std::cout << std::endl;
    }

    std::cout << "** Test ViSP to Yarp image conversion without copy" << std::endl;
    {
      bool convert_by_copy = false;
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
      std::cout << "   Reading the gray image with ViSP: " << filename << std::endl;
      vpImage<unsigned char> I;
      // Read an image on a disk
      vpImageIo::read(I, filename);

      yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
      // Convert the vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>
      vpImageConvert::convert(I, Iyarp, convert_by_copy);
      // Write the image
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_without_copy.pgm");
      std::cout << "   Converted Yarp image saved in: " << filename << std::endl;
      yarp::sig::file::write(*Iyarp, filename, yarp::sig::file::FORMAT_PGM);

      std::cout << "   Reading the gray image with Yarp: " << filename << std::endl;
      yarp::sig::ImageOf< yarp::sig::PixelMono > *IIyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
      yarp::sig::file::read(*IIyarp, filename, yarp::sig::file::FORMAT_PGM);
      vpImage<unsigned char> II;
      vpImageConvert::convert(IIyarp, II, convert_by_copy);
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_without_copy_visp.pgm");
      std::cout << "   Converted image in ViSP saved in: " << filename << std::endl;
      vpImageIo::write(II, filename);
      if (I != II) {
        std::cout << "   Yarp gray conversion test failed" << std::endl;
        return EXIT_FAILURE;
      }
      else {
        std::cout << "   Yarp gray conversion test succeed" << std::endl;
      }
      std::cout << std::endl;
    }
    {
      bool convert_by_copy = false;
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
      std::cout << "   Reading the color image with ViSP: " << filename << std::endl;
      vpImage<vpRGBa> I;
      // Read an image on a disk
      vpImageIo::read(I, filename);

      yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
      // Convert the vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgba\>
      vpImageConvert::convert(I, Iyarp, convert_by_copy);
      // Write the image
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_without_copy.ppm");
      std::cout << "   Converted image saved in: " << filename << std::endl;
      yarp::sig::file::write(*Iyarp, filename, yarp::sig::file::FORMAT_PPM);

      std::cout << "   Reading the color image with Yarp: " << filename << std::endl;
      yarp::sig::ImageOf< yarp::sig::PixelRgba > *IIyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
      yarp::sig::file::read(*IIyarp, filename, yarp::sig::file::FORMAT_PPM);
      vpImage<vpRGBa> II;
      vpImageConvert::convert(IIyarp, II, convert_by_copy);
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_without_copy_visp.ppm");
      std::cout << "   Converted image in ViSP saved in: " << filename << std::endl;
      vpImageIo::write(II, filename);
      if (I != II) {
        std::cout << "   Yarp RGBa color conversion test failed" << std::endl;
        return EXIT_FAILURE;
      }
      else {
        std::cout << "   Yarp RGBa color conversion test succeed" << std::endl;
      }

      yarp::sig::ImageOf< yarp::sig::PixelRgb > *IIIyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgb >();
      // Convert the vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgb\>
      vpImageConvert::convert(I, IIIyarp);
      // Write the image
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_without_copy_rgb.ppm");
      std::cout << "   Converted RGB image saved in: " << filename << std::endl;
      yarp::sig::file::write(*Iyarp, filename, yarp::sig::file::FORMAT_PPM);

      std::cout << "   Reading the RGB color image with Yarp: " << filename << std::endl;
      yarp::sig::file::read(*IIIyarp, filename, yarp::sig::file::FORMAT_PPM);
      vpImage<vpRGBa> III;
      vpImageConvert::convert(IIIyarp, III);
      filename = vpIoTools::createFilePath(opath, "Klimt_yarp_without_copy_visp_rgb.ppm");
      std::cout << "   Converted RGB image in ViSP saved in: " << filename << std::endl;
      vpImageIo::write(II, filename);
      if (I != III) {
        std::cout << "   Yarp RGB color conversion test failed" << std::endl;
        return EXIT_FAILURE;
      }
      else {
        std::cout << "   Yarp RGB color conversion test succeed" << std::endl;
      }

      std::cout << std::endl;
    }
#endif

    std::cout << "** All the tests succeed" << std::endl;

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
