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
 * Test contours extraction.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iomanip>

#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testContours.cpp

  \brief Test contours extraction.
*/

// List of allowed command line options
#define GETOPTARGS "cdi:o:h"

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
Test contours extraction.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>]\n\
     [-h]\n                 \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"Klimt/Klimt.pgm\"\n\
     image.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     output result images are written.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), opath.c_str(), user.c_str());

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
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'o':
      opath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, ipath, opath, user);
      return false;
      break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user);
      return false;
      break;
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

void printImage(const vpImage<unsigned char> &I, const std::string &name)
{
  std::cout << "\n" << name << ":" << std::endl;

  std::cout << "   ";
  for (unsigned int j = 0; j < I.getWidth(); j++) {
    std::cout << std::setfill(' ') << std::setw(2) << j << " ";
  }
  std::cout << std::endl;

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    std::cout << std::setfill(' ') << std::setw(2) << i << " ";

    for (unsigned int j = 0; j < I.getWidth(); j++) {
      std::cout << std::setfill(' ') << std::setw(2) << static_cast<unsigned int>(I[i][j]) << " ";
    }

    std::cout << std::endl;
  }
}

void displayContourInfo(const vp::vpContour &contour, const int level)
{
  std::cout << "\nContour:" << std::endl;
  std::cout << "\tlevel: " << level << std::endl;
  std::cout << "\tcontour type: " << (contour.m_contourType == vp::CONTOUR_OUTER ? "outer contour" : "hole contour")
            << std::endl;
  std::cout << "\tnb children: " << contour.m_children.size() << std::endl;

  for (std::vector<vp::vpContour *>::const_iterator it = contour.m_children.begin(); it != contour.m_children.end();
       ++it) {
    displayContourInfo(**it, level + 1);
  }
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
    if (getOptions(argc, argv, opt_ipath, opt_opath, username) == false) {
      exit(EXIT_FAILURE);
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
      } catch (...) {
        usage(argv[0], NULL, ipath, opt_opath, username);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << opath << std::endl;
        std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
        exit(EXIT_FAILURE);
      }
    }

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(EXIT_FAILURE);
    }

    //
    // Here starts really the test
    //

    unsigned char image_data[14 * 10] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    vpImage<unsigned char> I_test_data(image_data, 14, 10, true);
    std::cout << "Test with image data:" << std::endl;
    printImage(I_test_data, "I_test_data");

    vp::vpContour vp_contours;
    std::vector<std::vector<vpImagePoint> > contours;
    double t = vpTime::measureTimeMs();
    vp::findContours(I_test_data, vp_contours, contours);
    t = vpTime::measureTimeMs() - t;

    displayContourInfo(vp_contours, 0);
    std::cout << "ViSP: nb contours=" << contours.size() << " ; t=" << t << " ms" << std::endl;

    // Read Klimt.ppm
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpImage<unsigned char> I;
    std::cout << "Read image: " << filename << std::endl;
    vpImageIo::read(I, filename);
    vpImageTools::binarise(I, (unsigned char)127, (unsigned char)255, (unsigned char)0, (unsigned char)1,
                           (unsigned char)1);

    vpImage<unsigned char> I2(I.getHeight(), I.getWidth());
    for (unsigned int cpt = 0; cpt < I2.getSize(); cpt++) {
      I2.bitmap[cpt] = 255 * I.bitmap[cpt];
    }
    filename = vpIoTools::createFilePath(opath, "Klimt_contours_binarise.pgm");
    vpImageIo::write(I2, filename);

    t = vpTime::measureTimeMs();
    vp::findContours(I, vp_contours, contours);
    t = vpTime::measureTimeMs() - t;

    displayContourInfo(vp_contours, 0);
    std::cout << "\nTest with Klimt image:" << std::endl;
    std::cout << "ViSP: nb contours=" << contours.size() << " ; t=" << t << " ms" << std::endl;

    // Draw and save
    vpImage<unsigned char> I_draw_contours(I2.getHeight(), I2.getWidth(), 0);
    vp::drawContours(I_draw_contours, contours);

    filename = vpIoTools::createFilePath(opath, "Klimt_contours_extracted.pgm");
    vpImageIo::write(I_draw_contours, filename);

    vpImage<vpRGBa> I_draw_contours_color(I2.getHeight(), I2.getWidth(), vpRGBa(0, 0, 0));
    vp::drawContours(I_draw_contours_color, contours, vpColor::red);

    filename = vpIoTools::createFilePath(opath, "Klimt_contours_extracted_color.ppm");
    vpImageIo::write(I_draw_contours_color, filename);

    // Test retrieve list
    vp::findContours(I, vp_contours, contours, vp::CONTOUR_RETR_LIST);
    vpImage<unsigned char> I_draw_contours_list(I2.getHeight(), I2.getWidth(), 0);

    vpImage<unsigned char> I_tmp_list(I.getHeight(), I.getWidth(), 0);
    vp::drawContours(I_tmp_list, contours);

    contours.clear();
    for (std::vector<vp::vpContour *>::const_iterator it = vp_contours.m_children.begin();
         it != vp_contours.m_children.end(); ++it) {
      contours.push_back((*it)->m_points);
    }

    vp::drawContours(I_draw_contours_list, contours);
    std::cout << "(I_tmp_list == I_draw_contours_list)? " << (I_tmp_list == I_draw_contours_list) << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_contours_extracted_list.pgm");
    vpImageIo::write(I_draw_contours_list, filename);

    // Test retrieve external
    vp::findContours(I, vp_contours, contours, vp::CONTOUR_RETR_EXTERNAL);
    vpImage<unsigned char> I_draw_contours_external(I2.getHeight(), I2.getWidth(), 0);

    vpImage<unsigned char> I_tmp_external(I.getHeight(), I.getWidth(), 0);
    vp::drawContours(I_tmp_external, contours);

    contours.clear();
    for (std::vector<vp::vpContour *>::const_iterator it = vp_contours.m_children.begin();
         it != vp_contours.m_children.end(); ++it) {
      contours.push_back((*it)->m_points);
    }

    vp::drawContours(I_draw_contours_external, contours);
    std::cout << "(I_tmp_external == I_draw_contours_external)? " << (I_tmp_external == I_draw_contours_external)
              << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_contours_extracted_external.pgm");
    vpImageIo::write(I_draw_contours_external, filename);

    // Test fillHoles
    vpImage<unsigned char> I_holes = I_draw_contours_external;
    vpImageTools::binarise(I_holes, (unsigned char)127, (unsigned char)255, (unsigned char)0, (unsigned char)255,
                           (unsigned char)255);

    t = vpTime::measureTimeMs();
    vp::fillHoles(I_holes);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nFill Holes: " << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_contours_extracted_external_fill_holes.pgm");
    vpImageIo::write(I_holes, filename);

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    cv::Mat matImg;
    vpImageConvert::convert(I, matImg);

    std::vector<std::vector<cv::Point> > contours_opencv;
    double t_opencv = vpTime::measureTimeMs();
    cv::findContours(matImg, contours_opencv, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    t_opencv = vpTime::measureTimeMs() - t_opencv;
    std::cout << "\nOpenCV: nb contours=" << contours_opencv.size() << " ; t_opencv=" << t_opencv << " ms" << std::endl;

    vpImage<unsigned char> I_draw_contours_opencv(I.getHeight(), I.getWidth(), 0);
    for (std::vector<std::vector<cv::Point> >::const_iterator it1 = contours_opencv.begin();
         it1 != contours_opencv.end(); ++it1) {
      for (std::vector<cv::Point>::const_iterator it2 = it1->begin(); it2 != it1->end(); ++it2) {
        I_draw_contours_opencv[it2->y][it2->x] = 255;
      }
    }

    std::cout << "(I_draw_contours_opencv == I_drawContours)? " << (I_draw_contours_opencv == I_draw_contours)
              << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_contours_extracted_opencv.pgm");
    vpImageIo::write(I_draw_contours_opencv, filename);

    // Test retrieve list
    vpImageConvert::convert(I, matImg);
    contours_opencv.clear();
    cv::findContours(matImg, contours_opencv, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    I_draw_contours_opencv = 0;
    for (std::vector<std::vector<cv::Point> >::const_iterator it1 = contours_opencv.begin();
         it1 != contours_opencv.end(); ++it1) {
      for (std::vector<cv::Point>::const_iterator it2 = it1->begin(); it2 != it1->end(); ++it2) {
        I_draw_contours_opencv[it2->y][it2->x] = 255;
      }
    }

    std::cout << "(I_draw_contours_opencv == I_draw_contours_list)? "
              << (I_draw_contours_opencv == I_draw_contours_list) << std::endl;

    // Test retrieve external
    vpImageConvert::convert(I, matImg);
    contours_opencv.clear();
    cv::findContours(matImg, contours_opencv, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    I_draw_contours_opencv = 0;
    for (std::vector<std::vector<cv::Point> >::const_iterator it1 = contours_opencv.begin();
         it1 != contours_opencv.end(); ++it1) {
      for (std::vector<cv::Point>::const_iterator it2 = it1->begin(); it2 != it1->end(); ++it2) {
        I_draw_contours_opencv[it2->y][it2->x] = 255;
      }
    }

    std::cout << "(I_draw_contours_opencv == I_draw_contours_external)? "
              << (I_draw_contours_opencv == I_draw_contours_external) << std::endl;
#endif

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
