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
 * Test connected components.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
#include <map>
#include <set>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testConnectedComponents.cpp

  \brief Test connected components.
*/

// List of allowed command line options
#define GETOPTARGS "cdi:o:h"

void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, std::string user);
bool checkLabels(const vpImage<int> &label1, const vpImage<int> &label2);

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
Test connected components.\n\
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

bool checkLabels(const vpImage<int> &label1, const vpImage<int> &label2)
{
  if (label1.getHeight() != label2.getHeight() || label1.getWidth() != label2.getWidth())
    return false;

  std::map<int, std::vector<vpImagePoint> > map_label1, map_label2;
  for (unsigned int i = 0; i < label1.getHeight(); i++) {
    for (unsigned int j = 0; j < label1.getWidth(); j++) {
      if ((label1[i][j] > 0 && label2[i][j] == 0) || (label1[i][j] == 0 && label2[i][j] > 0)) {
        std::cerr << "label1[i][j] > 0 && label2[i][j] == 0 || label1[i][j] "
                     "== 0 && label2[i][j] > 0"
                  << std::endl;
        return false;
      }

      if (label1[i][j])
        map_label1[label1[i][j]].push_back(vpImagePoint(i, j));

      if (label2[i][j])
        map_label2[label2[i][j]].push_back(vpImagePoint(i, j));
    }
  }

  if (map_label1.size() != map_label2.size()) {
    std::cerr << "map_label1.size() != map_label2.size()" << std::endl;
    return false;
  }

  for (std::map<int, std::vector<vpImagePoint> >::const_iterator it1 = map_label1.begin(); it1 != map_label1.end();
       ++it1) {
    // Get corresponding label in the other method
    unsigned int i = (unsigned int)it1->second.front().get_i(), j = (unsigned int)it1->second.front().get_j();
    int lab2 = label2[i][j];

    std::vector<vpImagePoint>::const_iterator it_pt1 = it1->second.begin();
    for (; it_pt1 != it1->second.end(); ++it_pt1) {
      i = (unsigned int)it_pt1->get_i();
      j = (unsigned int)it_pt1->get_j();
      if (label2[i][j] != lab2) {
        std::cerr << "label2[i][j] != lab2" << std::endl;
        return false;
      }
    }
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

    // Read Klimt.ppm
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpImage<unsigned char> I;
    std::cout << "Read image: " << filename << std::endl;
    vpImageIo::read(I, filename);
    vpImageTools::binarise(I, (unsigned char)127, (unsigned char)255, (unsigned char)0, (unsigned char)255,
                           (unsigned char)255);
    std::cout << "Image: " << I.getWidth() << "x" << I.getHeight() << std::endl;

    vpImage<int> labels_connex4;
    int nbComponents = 0;
    double t = vpTime::measureTimeMs();
    vp::connectedComponents(I, labels_connex4, nbComponents, vpImageMorphology::CONNEXITY_4);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\n4-connexity connected components:" << std::endl;
    std::cout << "Time: " << t << " ms" << std::endl;
    std::cout << "nbComponents=" << nbComponents << std::endl;

    vpImage<int> labels_connex8;
    t = vpTime::measureTimeMs();
    vp::connectedComponents(I, labels_connex8, nbComponents, vpImageMorphology::CONNEXITY_8);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\n8-connexity connected components:" << std::endl;
    std::cout << "Time: " << t << " ms" << std::endl;
    std::cout << "nbComponents=" << nbComponents << std::endl;

    // Save results
    vpImage<vpRGBa> labels_connex4_color(labels_connex4.getHeight(), labels_connex4.getWidth(), vpRGBa(0, 0, 0, 0));
    for (unsigned int i = 0; i < labels_connex4.getHeight(); i++) {
      for (unsigned int j = 0; j < labels_connex4.getWidth(); j++) {
        if (labels_connex4[i][j] != 0) {
          labels_connex4_color[i][j] = vpRGBa(vpColor::getColor((unsigned int)labels_connex4[i][j]).R,
                                              vpColor::getColor((unsigned int)labels_connex4[i][j]).G,
                                              vpColor::getColor((unsigned int)labels_connex4[i][j]).B);
        }
      }
    }

    filename = vpIoTools::createFilePath(opath, "Klimt_connected_components_4.ppm");
    vpImageIo::write(labels_connex4_color, filename);

    vpImage<vpRGBa> labels_connex8_color(labels_connex8.getHeight(), labels_connex8.getWidth(), vpRGBa(0, 0, 0, 0));
    for (unsigned int i = 0; i < labels_connex8.getHeight(); i++) {
      for (unsigned int j = 0; j < labels_connex8.getWidth(); j++) {
        if (labels_connex8[i][j] != 0) {
          labels_connex8_color[i][j] = vpRGBa(vpColor::getColor((unsigned int)labels_connex8[i][j]).R,
                                              vpColor::getColor((unsigned int)labels_connex8[i][j]).G,
                                              vpColor::getColor((unsigned int)labels_connex8[i][j]).B);
        }
      }
    }

    filename = vpIoTools::createFilePath(opath, "Klimt_connected_components_8.ppm");
    vpImageIo::write(labels_connex8_color, filename);

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    cv::Mat matImg;
    vpImageConvert::convert(I, matImg);

    cv::Mat matLabels_4;
    double t_opencv = vpTime::measureTimeMs();
    cv::connectedComponents(matImg, matLabels_4, 4);
    t_opencv = vpTime::measureTimeMs() - t_opencv;

    std::set<int> set_labels_connex4_opencv;
    vpImage<int> labels_connex4_opencv((unsigned int)matLabels_4.rows, (unsigned int)matLabels_4.cols);
    for (int i = 0; i < matLabels_4.rows; i++) {
      for (int j = 0; j < matLabels_4.cols; j++) {
        labels_connex4_opencv[i][j] = matLabels_4.at<int>(i, j);

        if (matLabels_4.at<int>(i, j))
          set_labels_connex4_opencv.insert(matLabels_4.at<int>(i, j));
      }
    }

    std::cout << "\n4-connexity connected components (OpenCV):" << std::endl;
    std::cout << "Time: " << t_opencv << " ms" << std::endl;
    std::cout << "nb components: " << set_labels_connex4_opencv.size() << std::endl;
    bool check_label = checkLabels(labels_connex4_opencv, labels_connex4);
    std::cout << "checkLabels(labels_connex4_opencv, labels_connex4): " << check_label << std::endl;
    //    std::cout << "(labels_connex4_opencv == labels_connex4)? " <<
    //    (labels_connex4_opencv == labels_connex4) << std::endl;
    if (!check_label) {
      throw vpException(vpException::fatalError, "(labels_connex4_opencv != labels_connex4)");
    }

    cv::Mat matLabels_8;
    t_opencv = vpTime::measureTimeMs();
    cv::connectedComponents(matImg, matLabels_8, 8);
    t_opencv = vpTime::measureTimeMs() - t_opencv;

    std::set<int> set_labels_connex8_opencv;
    vpImage<int> labels_connex8_opencv((unsigned int)matLabels_8.rows, (unsigned int)matLabels_8.cols);
    for (int i = 0; i < matLabels_8.rows; i++) {
      for (int j = 0; j < matLabels_8.cols; j++) {
        labels_connex8_opencv[i][j] = matLabels_8.at<int>(i, j);

        if (matLabels_8.at<int>(i, j))
          set_labels_connex8_opencv.insert(matLabels_8.at<int>(i, j));
      }
    }

    std::cout << "\n8-connexity connected components (OpenCV):" << std::endl;
    std::cout << "nb components: " << set_labels_connex8_opencv.size() << std::endl;
    std::cout << "Time: " << t_opencv << " ms" << std::endl;
    check_label = checkLabels(labels_connex8_opencv, labels_connex8);
    std::cout << "checkLabels(labels_connex8_opencv, labels_connex8): " << check_label << std::endl;
    //    std::cout << "(labels_connex8_opencv == labels_connex8)? " <<
    //    (labels_connex8_opencv == labels_connex8) << std::endl;

    if (!check_label) {
      throw vpException(vpException::fatalError, "(labels_connex8_opencv != labels_connex8)");
    }
#endif

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
