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
 * Test saving / loading learning files for vpKeyPoint class.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iomanip>
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpKeyPoint.h>

// List of allowed command line options
#define GETOPTARGS "cdo:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam, const std::string &opath, const std::string &user)
{
  fprintf(stdout, "\n\
Test save / load learning files for vpKeyPoint class.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
\n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     learning files will be written.\n\
\n\
  -h\n\
     Print the help.\n", opath.c_str(), user.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &opath, const std::string &user)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      break; // not used, to avoid error with default arguments ctest
    case 'd':
      break; // not used, to avoid error with default arguments ctest
    case 'o':
      opath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, opath, user);
      return false;
      break;

    default:
      usage(argv[0], optarg_, opath, user);
      return false;
      break;
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  Compare two vectors of cv::KeyPoint.

  \param keypoints1 : First vectors of cv::KeyPoint.
  \param keypoints2 : Second vectors of cv::KeyPoint.

  \return True if the two vectors are identical, false otherwise.
*/
bool compareKeyPoints(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2)
{
  if (keypoints1.size() != keypoints2.size()) {
    return false;
  }

  for (size_t cpt = 0; cpt < keypoints1.size(); cpt++) {
    if (!vpMath::equal(keypoints1[cpt].angle, keypoints2[cpt].angle, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].angle=" << keypoints1[cpt].angle
                << " ; keypoints2[cpt].angle=" << keypoints2[cpt].angle << std::endl;
      return false;
    }

    if (keypoints1[cpt].class_id != keypoints2[cpt].class_id) {
      std::cerr << "keypoints1[cpt].class_id=" << keypoints1[cpt].class_id
                << " ; keypoints2[cpt].class_id=" << keypoints2[cpt].class_id << std::endl;
      return false;
    }

    if (keypoints1[cpt].octave != keypoints2[cpt].octave) {
      std::cerr << "keypoints1[cpt].octave=" << keypoints1[cpt].octave
                << " ; keypoints2[cpt].octave=" << keypoints2[cpt].octave << std::endl;
      return false;
    }

    if (!vpMath::equal(keypoints1[cpt].pt.x, keypoints2[cpt].pt.x, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].pt.x=" << keypoints1[cpt].pt.x
                << " ; keypoints2[cpt].pt.x=" << keypoints2[cpt].pt.x << std::endl;
      return false;
    }

    if (!vpMath::equal(keypoints1[cpt].pt.y, keypoints2[cpt].pt.y, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].pt.y=" << keypoints1[cpt].pt.y
                << " ; keypoints2[cpt].pt.y=" << keypoints2[cpt].pt.y << std::endl;
      return false;
    }

    if (!vpMath::equal(keypoints1[cpt].response, keypoints2[cpt].response, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].response=" << keypoints1[cpt].response
                << " ; keypoints2[cpt].response=" << keypoints2[cpt].response << std::endl;
      return false;
    }

    if (!vpMath::equal(keypoints1[cpt].size, keypoints2[cpt].size, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].size=" << keypoints1[cpt].size
                << " ; keypoints2[cpt].size=" << keypoints2[cpt].size << std::endl;
      return false;
    }
  }

  return true;
}

/*!
  Compare two descriptors.

  \param descriptors1 : First descriptor.
  \param descriptors2 : Second descriptor.

  \return True if the two vectors are identical, false otherwise.
*/
bool compareDescriptors(const cv::Mat &descriptors1, const cv::Mat &descriptors2)
{
  if (descriptors1.rows != descriptors2.rows || descriptors1.cols != descriptors2.cols ||
      descriptors1.type() != descriptors2.type()) {
    return false;
  }

  for (int i = 0; i < descriptors1.rows; i++) {
    for (int j = 0; j < descriptors1.cols; j++) {
      switch (descriptors1.type()) {
      case CV_8U:
        if (descriptors1.at<unsigned char>(i, j) != descriptors2.at<unsigned char>(i, j)) {
          std::cerr << "descriptors1.at<unsigned char>(i,j)=" << descriptors1.at<unsigned char>(i, j)
                    << " ; descriptors2.at<unsigned char>(i,j)=" << descriptors2.at<unsigned char>(i, j) << std::endl;
          return false;
        }
        break;

      case CV_8S:
        if (descriptors1.at<char>(i, j) != descriptors2.at<char>(i, j)) {
          std::cerr << "descriptors1.at<char>(i,j)=" << descriptors1.at<char>(i, j)
                    << " ; descriptors2.at<char>(i,j)=" << descriptors2.at<char>(i, j) << std::endl;
          return false;
        }
        break;

      case CV_16U:
        if (descriptors1.at<unsigned short>(i, j) != descriptors2.at<unsigned short>(i, j)) {
          std::cerr << "descriptors1.at<unsigned short>(i,j)=" << descriptors1.at<unsigned short>(i, j)
                    << " ; descriptors2.at<unsigned short>(i,j)=" << descriptors2.at<unsigned short>(i, j) << std::endl;
          return false;
        }
        break;

      case CV_16S:
        if (descriptors1.at<short>(i, j) != descriptors2.at<short>(i, j)) {
          std::cerr << "descriptors1.at<short>(i,j)=" << descriptors1.at<short>(i, j)
                    << " ; descriptors2.at<short>(i,j)=" << descriptors2.at<short>(i, j) << std::endl;
          return false;
        }
        break;

      case CV_32S:
        if (descriptors1.at<int>(i, j) != descriptors2.at<int>(i, j)) {
          std::cerr << "descriptors1.at<int>(i,j)=" << descriptors1.at<int>(i, j)
                    << " ; descriptors2.at<int>(i,j)=" << descriptors2.at<int>(i, j) << std::endl;
          return false;
        }
        break;

      case CV_32F:
        if (!vpMath::equal(descriptors1.at<float>(i, j), descriptors2.at<float>(i, j),
                           std::numeric_limits<float>::epsilon())) {
          std::cerr << std::fixed << std::setprecision(9)
                    << "descriptors1.at<float>(i,j)=" << descriptors1.at<float>(i, j)
                    << " ; descriptors2.at<float>(i,j)=" << descriptors2.at<float>(i, j) << std::endl;
          return false;
        }
        break;

      case CV_64F:
        if (!vpMath::equal(descriptors1.at<double>(i, j), descriptors2.at<double>(i, j),
                           std::numeric_limits<double>::epsilon())) {
          std::cerr << std::fixed << std::setprecision(17)
                    << "descriptors1.at<double>(i,j)=" << descriptors1.at<double>(i, j)
                    << " ; descriptors2.at<double>(i,j)=" << descriptors2.at<double>(i, j) << std::endl;
          return false;
        }
        break;

      default:
        return false;
        break;
      }
    }
  }

  return true;
}

template<typename Type>
void run_test(const std::string &env_ipath, const std::string &opath,  vpImage<Type> &I)
{
  std::string filename;
  // Set the path location of the image sequence
  std::string dirname = vpIoTools::createFilePath(env_ipath, "Klimt");

  // Build the name of the image files
  std::string img_filename = vpIoTools::createFilePath(dirname, "/Klimt.ppm");
  vpImageIo::read(I, img_filename);

  vpKeyPoint keyPoints;

  // Test with binary descriptor
  {
    std::string keypointName = "ORB";
    keyPoints.setDetector(keypointName);
    keyPoints.setExtractor(keypointName);

    keyPoints.buildReference(I);

    std::vector<cv::KeyPoint> trainKeyPoints;
    keyPoints.getTrainKeyPoints(trainKeyPoints);
    cv::Mat trainDescriptors = keyPoints.getTrainDescriptors();
    if (trainKeyPoints.empty() || trainDescriptors.empty() || (int)trainKeyPoints.size() != trainDescriptors.rows) {
      throw vpException(vpException::fatalError, "Problem when detecting "
                                                 "keypoints or when "
                                                 "computing descriptors !");
    }

    // Save in binary with training images
    filename = vpIoTools::createFilePath(opath, "bin_with_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_bin_with_img.bin");
    keyPoints.saveLearningData(filename, true, true);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint1;
    read_keypoint1.loadLearningData(filename, true);
    std::vector<cv::KeyPoint> trainKeyPoints_read;
    read_keypoint1.getTrainKeyPoints(trainKeyPoints_read);
    cv::Mat trainDescriptors_read = read_keypoint1.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved "
                                                 "in binary with train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "binary with train images saved !");
    }

    // Save in binary with no training images
    filename = vpIoTools::createFilePath(opath, "bin_without_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_bin_without_img.bin");
    keyPoints.saveLearningData(filename, true, false);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint2;
    read_keypoint2.loadLearningData(filename, true);
    trainKeyPoints_read.clear();
    read_keypoint2.getTrainKeyPoints(trainKeyPoints_read);
    trainDescriptors_read = read_keypoint2.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "binary without train images !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "binary without train images !");
    }

#if defined(VISP_HAVE_XML2)
    // Save in xml with training images
    filename = vpIoTools::createFilePath(opath, "xml_with_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_xml_with_img.xml");
    keyPoints.saveLearningData(filename, false, true);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint3;
    read_keypoint3.loadLearningData(filename, false);
    trainKeyPoints_read.clear();
    read_keypoint3.getTrainKeyPoints(trainKeyPoints_read);
    trainDescriptors_read = read_keypoint3.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "xml with train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "xml with train images saved !");
    }

    // Save in xml without training images
    filename = vpIoTools::createFilePath(opath, "xml_without_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_xml_without_img.xml");
    keyPoints.saveLearningData(filename, false, false);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint4;
    read_keypoint4.loadLearningData(filename, false);
    trainKeyPoints_read.clear();
    read_keypoint4.getTrainKeyPoints(trainKeyPoints_read);
    trainDescriptors_read = read_keypoint4.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "xml without train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "xml without train images saved !");
    }
#endif

    std::cout << "Saving / loading learning files with binary descriptor are ok !" << std::endl;
  }

// Test with floating point descriptor
#if defined(VISP_HAVE_OPENCV_NONFREE) ||                                                                               \
  ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(VISP_HAVE_OPENCV_XFEATURES2D))
  {
    std::string keypointName = "SIFT";
    keyPoints.setDetector(keypointName);
    keyPoints.setExtractor(keypointName);

    keyPoints.buildReference(I);

    std::vector<cv::KeyPoint> trainKeyPoints;
    keyPoints.getTrainKeyPoints(trainKeyPoints);
    cv::Mat trainDescriptors = keyPoints.getTrainDescriptors();
    if (trainKeyPoints.empty() || trainDescriptors.empty() || (int)trainKeyPoints.size() != trainDescriptors.rows) {
      throw vpException(vpException::fatalError, "Problem when detecting keypoints or when "
                                                 "computing descriptors (SIFT) !");
    }

    // Save in binary with training images
    filename = vpIoTools::createFilePath(opath, "bin_with_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_bin_with_img.bin");
    keyPoints.saveLearningData(filename, true, true);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint1;
    read_keypoint1.loadLearningData(filename, true);
    std::vector<cv::KeyPoint> trainKeyPoints_read;
    read_keypoint1.getTrainKeyPoints(trainKeyPoints_read);
    cv::Mat trainDescriptors_read = read_keypoint1.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "binary with train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "binary with train images saved !");
    }

    // Save in binary with no training images
    filename = vpIoTools::createFilePath(opath, "bin_without_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_bin_without_img.bin");
    keyPoints.saveLearningData(filename, true, false);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint2;
    read_keypoint2.loadLearningData(filename, true);
    trainKeyPoints_read.clear();
    read_keypoint2.getTrainKeyPoints(trainKeyPoints_read);
    trainDescriptors_read = read_keypoint2.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "binary without train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "binary without train images saved !");
    }

#if defined(VISP_HAVE_XML2)
    // Save in xml with training images
    filename = vpIoTools::createFilePath(opath, "xml_with_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_xml_with_img.xml");
    keyPoints.saveLearningData(filename, false, true);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint3;
    read_keypoint3.loadLearningData(filename, false);
    trainKeyPoints_read.clear();
    read_keypoint3.getTrainKeyPoints(trainKeyPoints_read);
    trainDescriptors_read = read_keypoint3.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "xml with train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "xml with train images saved !");
    }

    // Save in xml without training images
    filename = vpIoTools::createFilePath(opath, "xml_without_img");
    vpIoTools::makeDirectory(filename);
    filename = vpIoTools::createFilePath(filename, "test_save_in_xml_without_img.xml");
    keyPoints.saveLearningData(filename, false, false);

    // Test if save is ok
    if (!vpIoTools::checkFilename(filename)) {
      std::stringstream ss;
      ss << "Problem when saving file=" << filename;
      throw vpException(vpException::ioError, ss.str().c_str());
    }

    // Test if read is ok
    vpKeyPoint read_keypoint4;
    read_keypoint4.loadLearningData(filename, false);
    trainKeyPoints_read.clear();
    read_keypoint4.getTrainKeyPoints(trainKeyPoints_read);
    trainDescriptors_read = read_keypoint4.getTrainDescriptors();

    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_read)) {
      throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved in "
                                                 "xml without train images saved !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_read)) {
      throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading "
                                                 "learning file saved in "
                                                 "xml without train images saved !");
    }
#endif

    std::cout << "Saving / loading learning files with floating point "
                 "descriptor are ok !"
              << std::endl;

    // Test vpKeyPoint::reset()
    vpKeyPoint keypoint_reset;

    keypointName = "ORB";
    keypoint_reset.setDetector(keypointName);
    keypoint_reset.setExtractor(keypointName);

    keypoint_reset.buildReference(I);

    // reset
    keypoint_reset.reset();

    keypointName = "SIFT";
    keypoint_reset.setDetector(keypointName);
    keypoint_reset.setExtractor(keypointName);

    keypoint_reset.buildReference(I);

    std::vector<cv::KeyPoint> trainKeyPoints_reset;
    keypoint_reset.getTrainKeyPoints(trainKeyPoints_reset);
    cv::Mat trainDescriptors_reset = keypoint_reset.getTrainDescriptors();

    // If reset is ok, we should get the same keypoints and the same
    // descriptors
    if (!compareKeyPoints(trainKeyPoints, trainKeyPoints_reset)) {
      throw vpException(vpException::fatalError, "Problem with vpKeyPoint::reset() and trainKeyPoints !");
    }

    if (!compareDescriptors(trainDescriptors, trainDescriptors_reset)) {
      throw vpException(vpException::fatalError, "Problem with vpKeyPoint::reset() and trainDescriptors !");
    }

    std::cout << "vpKeyPoint::reset() is ok with trainKeyPoints and "
                 "trainDescriptors !"
              << std::endl;
  }
#endif
}

/*!
  \example testKeyPoint-7.cpp

  \brief   Test saving / loading learning file.
*/
int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_opath;
    std::string username;
    std::string opath;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if (env_ipath.empty()) {
      throw vpException(vpException::ioError, "Please set the VISP_INPUT_IMAGE_PATH environment variable value.");
    }

// Set the default output path
#if defined(_WIN32)
    opt_opath = "C:/temp";
#else
    opt_opath = "/tmp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_opath, username) == false) {
      throw vpException(vpException::fatalError, "getOptions(argc, argv, opt_opath, username) == false");
    }

    // Get the option values
    if (!opt_opath.empty()) {
      opath = opt_opath;
    }

    // Append to the output path string, the login name of the user
    opath = vpIoTools::createFilePath(opath, username);

    {
      vpImage<unsigned char> I;

      std::cout << "-- Test on gray level images" << std::endl;
      run_test(env_ipath, opath, I);
    }

    {
      vpImage<vpRGBa> I;

      std::cout << "-- Test on color images" << std::endl;
      run_test(env_ipath, opath, I);
    }

  } catch (const vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "Saving / loading learning files are ok !" << std::endl;
  std::cout << "testKeyPoint-7 is ok !" << std::endl;
  return 0;
}
#else
int main()
{
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
