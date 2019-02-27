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
 * Test some functions from vpImageFilter class.
 *
 *****************************************************************************/
/*!
  \example testImageFilter.cpp

  \brief Test some functions from vpImageFilter class.
*/

#include <iostream>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
#include <opencv2/imgproc/imgproc.hpp>
#endif

// List of allowed command line options
#define GETOPTARGS "cdi:p:h"

namespace
{
/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
 */
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
  Test vpImageFilter class.\n\
  \n\
  SYNOPSIS\n\
    %s [-i <input image path>] [-p <personal image path>]\n\
       [-h]\n            \
  ", name);

  fprintf(stdout, "\n\
  OPTIONS:                                               Default\n\
    -i <input image path>                                %s\n\
       Set image input path.\n\
       From this path read \"Klimt/Klimt.pgm,\n\
       .ppm, .jpeg and .png images.\n\
       Setting the VISP_INPUT_IMAGE_PATH environment\n\
       variable produces the same behaviour than using\n\
       this option.\n\
  \n\
    -p <personal image path>                               \n\
       Path to an image used to test image reading function.\n\
       Example: -p /my_path_to/image.png\n\
  \n\
    -h\n\
       Print the help.\n\n", ipath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param ppath : Personal image path.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'p':
      ppath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, ipath);
      return false;
      break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
bool check_results(const cv::Mat &mat, const vpImage<double> &I, const unsigned int half_size_y,
                   const unsigned int half_size_x)
{
  for (unsigned int i = half_size_y; i < I.getHeight() - half_size_y; i++) {
    for (unsigned int j = half_size_x; j < I.getWidth() - half_size_x; j++) {
      if (!vpMath::equal(mat.at<double>(static_cast<int>(i), static_cast<int>(j)), I[i][j], std::numeric_limits<double>::epsilon())) {
        return false;
      }
    }
  }

  return true;
}

bool check_results(const cv::Mat &mat, const vpImage<double> &I, unsigned int margin, double threshold)
{
  for (unsigned int i = margin; i < I.getHeight() - margin; i++) {
    for (unsigned int j = margin; j < I.getWidth() - margin; j++) {
      if (!vpMath::equal(mat.at<unsigned char>(static_cast<int>(i), static_cast<int>(j)), I[i][j], threshold)) {
        return false;
      }
    }
  }

  return true;
}

bool check_results(const cv::Mat &mat, const vpImage<vpRGBa> &I, unsigned int margin, double threshold)
{
  for (unsigned int i = margin; i < I.getHeight() - margin; i++) {
    for (unsigned int j = margin; j < I.getWidth() - margin; j++) {
      if (!vpMath::equal(static_cast<double>(mat.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j))[2]), I[i][j].R, threshold)) {
        return false;
      }
      if (!vpMath::equal(static_cast<double>(mat.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j))[1]), I[i][j].G, threshold)) {
        return false;
      }
      if (!vpMath::equal(static_cast<double>(mat.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j))[0]), I[i][j].B, threshold)) {
        return false;
      }
    }
  }

  return true;
}
#endif
}

int main(int argc, const char *argv[])
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string opt_ppath;
    std::string ipath;
    std::string filename;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath) == false) {
      exit(EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

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

    //
    // Here starts really the test
    //

    // Test on small images first
    vpImage<unsigned char> I(6, 6);
    for (unsigned int i = 0; i < I.getSize(); i++) {
      I.bitmap[i] = (unsigned char)i;
    }
    std::cout << "I:\n" << I << std::endl;

    vpMatrix kernel_1(2, 2);
    for (unsigned int i = 0, cpt = 1; i < kernel_1.getRows(); i++) {
      for (unsigned int j = 0; j < kernel_1.getCols(); j++, cpt++) {
        kernel_1[i][j] = cpt;
      }
    }
    std::cout << "kernel_1:\n" << kernel_1 << std::endl;

    vpMatrix kernel_2(3, 3);
    for (unsigned int i = 0, cpt = 1; i < kernel_2.getRows(); i++) {
      for (unsigned int j = 0; j < kernel_2.getCols(); j++, cpt++) {
        kernel_2[i][j] = cpt;
      }
    }
    std::cout << "kernel_2:\n" << kernel_2 << std::endl;

    vpMatrix kernel_3(2, 3);
    for (unsigned int i = 0, cpt = 1; i < kernel_3.getRows(); i++) {
      for (unsigned int j = 0; j < kernel_3.getCols(); j++, cpt++) {
        kernel_3[i][j] = cpt;
      }
    }
    std::cout << "kernel_3:\n" << kernel_3 << std::endl;

    // Test correlation
    vpImage<double> I_correlation_1, I_correlation_2, I_correlation_3;
    vpImageFilter::filter(I, I_correlation_1, kernel_1);
    vpImageFilter::filter(I, I_correlation_2, kernel_2);
    vpImageFilter::filter(I, I_correlation_3, kernel_3);

    std::cout << "\nI_correlation_1:\n" << I_correlation_1 << std::endl;
    std::cout << "I_correlation_2:\n" << I_correlation_2 << std::endl;
    std::cout << "I_correlation_3:\n" << I_correlation_3 << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cv::Mat matImg;
    vpImageConvert::convert(I, matImg);

    cv::Mat mat_kernel_1(2, 2, CV_64F);
    for (int i = 0, cpt = 1; i < mat_kernel_1.rows; i++) {
      for (int j = 0; j < mat_kernel_1.cols; j++, cpt++) {
        mat_kernel_1.at<double>(i, j) = cpt;
      }
    }

    cv::Mat mat_kernel_2(3, 3, CV_64F);
    for (int i = 0, cpt = 1; i < mat_kernel_2.rows; i++) {
      for (int j = 0; j < mat_kernel_2.cols; j++, cpt++) {
        mat_kernel_2.at<double>(i, j) = cpt;
      }
    }

    cv::Mat mat_kernel_3(2, 3, CV_64F);
    for (int i = 0, cpt = 1; i < mat_kernel_3.rows; i++) {
      for (int j = 0; j < mat_kernel_3.cols; j++, cpt++) {
        mat_kernel_3.at<double>(i, j) = cpt;
      }
    }

    cv::Mat matImg_correlation_1, matImg_correlation_2, matImg_correlation_3;
    cv::filter2D(matImg, matImg_correlation_1, CV_64F, mat_kernel_1);
    cv::filter2D(matImg, matImg_correlation_2, CV_64F, mat_kernel_2);
    cv::filter2D(matImg, matImg_correlation_3, CV_64F, mat_kernel_3);

    std::cout << "\nTest correlation on small image:" << std::endl;
    std::cout << "(I_correlation_1 == matImg_correlation_1)? "
              << check_results(matImg_correlation_1, I_correlation_1, kernel_1.getRows() / 2, kernel_1.getCols() / 2)
              << std::endl;
    std::cout << "(I_correlation_2 == matImg_correlation_2)? "
              << check_results(matImg_correlation_2, I_correlation_2, kernel_2.getRows() / 2, kernel_2.getCols() / 2)
              << std::endl;
    std::cout << "(I_correlation_3 == matImg_correlation_3)? "
              << check_results(matImg_correlation_3, I_correlation_3, kernel_3.getRows() / 2, kernel_3.getCols() / 2)
              << std::endl;
#endif

    // Test convolution
    vpImage<double> I_convolution_1, I_convolution_2, I_convolution_3;
    vpImageFilter::filter(I, I_convolution_1, kernel_1, true);
    vpImageFilter::filter(I, I_convolution_2, kernel_2, true);
    vpImageFilter::filter(I, I_convolution_3, kernel_3, true);

    std::cout << "\nI_convolution_1:\n" << I_convolution_1 << std::endl;
    std::cout << "I_convolution_2:\n" << I_convolution_2 << std::endl;
    std::cout << "I_convolution_3:\n" << I_convolution_3 << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cv::Mat mat_kernel_1_flip, mat_kernel_2_flip, mat_kernel_3_flip;
    cv::flip(mat_kernel_1, mat_kernel_1_flip, -1);
    cv::flip(mat_kernel_2, mat_kernel_2_flip, -1);
    cv::flip(mat_kernel_3, mat_kernel_3_flip, -1);

    cv::Mat matImg_convolution_1, matImg_convolution_2, matImg_convolution_3;

    cv::Point anchor1(mat_kernel_1_flip.cols - mat_kernel_1_flip.cols / 2 - 1,
                      mat_kernel_1_flip.rows - mat_kernel_1_flip.rows / 2 - 1);
    cv::filter2D(matImg, matImg_convolution_1, CV_64F, mat_kernel_1_flip, anchor1);

    cv::Point anchor2(mat_kernel_2_flip.cols - mat_kernel_2_flip.cols / 2 - 1,
                      mat_kernel_2_flip.rows - mat_kernel_2_flip.rows / 2 - 1);
    cv::filter2D(matImg, matImg_convolution_2, CV_64F, mat_kernel_2_flip, anchor2);

    cv::Point anchor3(mat_kernel_3_flip.cols - mat_kernel_3_flip.cols / 2 - 1,
                      mat_kernel_3_flip.rows - mat_kernel_3_flip.rows / 2 - 1);
    cv::filter2D(matImg, matImg_convolution_3, CV_64F, mat_kernel_3_flip, anchor3);

    std::cout << "\nTest convolution on small image:" << std::endl;
    std::cout << "(I_convolution_1 == matImg_convolution_1)? "
              << check_results(matImg_convolution_1, I_convolution_1, kernel_1.getRows() / 2, kernel_1.getCols() / 2)
              << std::endl;
    std::cout << "(I_convolution_2 == matImg_convolution_2)? "
              << check_results(matImg_convolution_2, I_convolution_2, kernel_2.getRows() / 2, kernel_2.getCols() / 2)
              << std::endl;
    std::cout << "(I_convolution_3 == matImg_convolution_3)? "
              << check_results(matImg_convolution_3, I_convolution_3, kernel_3.getRows() / 2, kernel_3.getCols() / 2)
              << std::endl;
#endif
    if (opt_ppath.empty()) {
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
      vpImageIo::read(I, filename);
    } else {
      filename = opt_ppath;
      vpImageIo::read(I, filename);
      printf("Image \"%s\" read successfully\n", filename.c_str());
    }

    // Test correlation
    double t = vpTime::measureTimeMs();
    vpImageFilter::filter(I, I_correlation_1, kernel_1);
    vpImageFilter::filter(I, I_correlation_2, kernel_2);
    vpImageFilter::filter(I, I_correlation_3, kernel_3);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nTime to do 3 correlation filtering: " << t << " ms ; Mean: " << t / 3.0 << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    vpImageConvert::convert(I, matImg);

    t = vpTime::measureTimeMs();
    cv::filter2D(matImg, matImg_correlation_1, CV_64F, mat_kernel_1);
    cv::filter2D(matImg, matImg_correlation_2, CV_64F, mat_kernel_2);
    cv::filter2D(matImg, matImg_correlation_3, CV_64F, mat_kernel_3);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do 3 cv::filter2D: " << t << " ms ; Mean: " << t / 3.0 << " ms" << std::endl;

    std::cout << "\nTest correlation on Klimt image:" << std::endl;
    bool test = check_results(matImg_correlation_1, I_correlation_1, kernel_1.getRows() / 2, kernel_1.getCols() / 2);
    std::cout << "(I_correlation_1 == matImg_correlation_1)? " << test << std::endl;
    if (!test) {
      std::cerr << "Failed test1 correlation with vpImageFilter::filter()!" << std::endl;
      return EXIT_FAILURE;
    }

    test = check_results(matImg_correlation_2, I_correlation_2, kernel_2.getRows() / 2, kernel_2.getCols() / 2);
    std::cout << "(I_correlation_2 == matImg_correlation_2)? " << test << std::endl;
    if (!test) {
      std::cerr << "Failed test2 correlation with vpImageFilter::filter()!" << std::endl;
      return EXIT_FAILURE;
    }

    test = check_results(matImg_correlation_3, I_correlation_3, kernel_3.getRows() / 2, kernel_3.getCols() / 2);
    std::cout << "(I_correlation_3 == matImg_correlation_3)? " << test << std::endl;
    if (!test) {
      std::cerr << "Failed test3 correlation with vpImageFilter::filter()!" << std::endl;
      return EXIT_FAILURE;
    }
#endif

    // Test convolution
    t = vpTime::measureTimeMs();
    vpImageFilter::filter(I, I_convolution_1, kernel_1, true);
    vpImageFilter::filter(I, I_convolution_2, kernel_2, true);
    vpImageFilter::filter(I, I_convolution_3, kernel_3, true);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nTime to do 3 convolution filtering: " << t << " ms ; Mean: " << t / 3.0 << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)

    t = vpTime::measureTimeMs();
    cv::filter2D(matImg, matImg_convolution_1, CV_64F, mat_kernel_1_flip, anchor1);
    cv::filter2D(matImg, matImg_convolution_2, CV_64F, mat_kernel_2_flip, anchor2);
    cv::filter2D(matImg, matImg_convolution_3, CV_64F, mat_kernel_3_flip, anchor3);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do 3 cv::filter2D: " << t << " ms ; Mean: " << t / 3.0 << " ms" << std::endl;

    std::cout << "\nTest convolution on Klimt image:" << std::endl;
    test = check_results(matImg_convolution_1, I_convolution_1, kernel_1.getRows() / 2, kernel_1.getCols() / 2);
    std::cout << "(I_convolution_1 == matImg_convolution_1)? " << test << std::endl;
    if (!test) {
      std::cerr << "Failed test1 convolution with vpImageFilter::filter()!" << std::endl;
      return EXIT_FAILURE;
    }

    test = check_results(matImg_convolution_2, I_convolution_2, kernel_2.getRows() / 2, kernel_2.getCols() / 2);
    std::cout << "(I_convolution_2 == matImg_convolution_2)? " << test << std::endl;
    if (!test) {
      std::cerr << "Failed test2 convolution with vpImageFilter::filter()!" << std::endl;
      return EXIT_FAILURE;
    }

    test = check_results(matImg_convolution_3, I_convolution_3, kernel_3.getRows() / 2, kernel_3.getCols() / 2);
    std::cout << "(I_convolution_3 == matImg_convolution_3)? " << test << std::endl;
    if (!test) {
      std::cerr << "Failed test3 convolution with vpImageFilter::filter()!" << std::endl;
      return EXIT_FAILURE;
    }
#endif

    // Test Sobel
    vpMatrix kernel_sobel_x_flip(5, 5);
    vpImageFilter::getSobelKernelX(kernel_sobel_x_flip.data, 2);
    vpMatrix kernel_sobel_x(5, 5);
    for (unsigned int i = 0; i < kernel_sobel_x.getRows(); i++) {
      for (unsigned int j = 0; j < kernel_sobel_x.getCols(); j++) {
        kernel_sobel_x[i][j] = kernel_sobel_x_flip[i][kernel_sobel_x.getCols()-1-j];
      }
    }

    vpImage<double> I_sobel_x;
    t = vpTime::measureTimeMs();
    vpImageFilter::filter(I, I_sobel_x, kernel_sobel_x, true);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nTime to do Sobel: " << t << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cv::Mat matImg_sobel_x;
    t = vpTime::measureTimeMs();
    cv::Sobel(matImg, matImg_sobel_x, CV_64F, 1, 0, 5);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do cv::Sobel: " << t << " ms" << std::endl;

    std::cout << "\nTest Sobel on Klimt image:" << std::endl;
    std::cout << "(I_sobel_x == matImg_sobel_x)? "
              << check_results(matImg_sobel_x, I_sobel_x, kernel_sobel_x.getRows() / 2, kernel_sobel_x.getCols() / 2)
              << std::endl;
#endif

    vpImage<double> I_double, Iu, Iv;
    vpImageConvert::convert(I, I_double);
    t = vpTime::measureTimeMs();
    vpImageFilter::filter(I_double, Iu, Iv, kernel_sobel_x, true);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nTime to do Sobel Iu and Iv: " << t << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cv::Mat matImg_sobel_y;
    cv::Sobel(matImg, matImg_sobel_y, CV_64F, 0, 1, 5);

    std::cout << "(Iu == matImg_sobel_x)? "
              << check_results(matImg_sobel_x, Iu, kernel_sobel_x.getRows() / 2, kernel_sobel_x.getCols() / 2)
              << std::endl;
    std::cout << "(Iv == matImg_sobel_y)? "
              << check_results(matImg_sobel_y, Iv, kernel_sobel_x.getRows() / 2, kernel_sobel_x.getCols() / 2)
              << std::endl;
#endif

    // Test Sobel separable filters
    vpImage<double> I_sep_filtered;
    vpColVector kernel_sep_x(5);
    kernel_sep_x[0] = 1.0;
    kernel_sep_x[1] = 2.0;
    kernel_sep_x[2] = 0.0;
    kernel_sep_x[3] = -2.0;
    kernel_sep_x[4] = -1.0;
    vpColVector kernel_sep_y(5);
    kernel_sep_y[0] = 1.0;
    kernel_sep_y[1] = 4.0;
    kernel_sep_y[2] = 6.0;
    kernel_sep_y[3] = 4.0;
    kernel_sep_y[4] = 1.0;

    t = vpTime::measureTimeMs();
    vpImageFilter::sepFilter(I, I_sep_filtered, kernel_sep_x, kernel_sep_y);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nTime to do sepFilter: " << t << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    test = check_results(matImg_sobel_x, Iu, I_sep_filtered.getRows() / 2, kernel_sobel_x.getCols() / 2);
    std::cout << "(I_sep_filtered == matImg_sobel_x)? " << test << std::endl;

    if (!test) {
      std::cerr << "Failed separable filter!" << std::endl;
      return EXIT_FAILURE;
    }
#endif

    {
      // Test Gaussian blur on grayscale image

      std::cout << "\nTest Gaussian Blur on Klimt grayscale image:" << std::endl;
      vpImage<unsigned char> I;
      vpImage<double> I_blur;
      // Test on real image

      if (opt_ppath.empty()) {
        filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
        vpImageIo::read(I, filename);
      } else {
        filename = opt_ppath;
        vpImageIo::read(I, filename);
        printf("Image \"%s\" read successfully\n", filename.c_str());
      }

      unsigned int gaussian_filter_size = 7;
      double sigma = 3;
      t = vpTime::measureTimeMs();
      vpImageFilter::gaussianBlur(I, I_blur, gaussian_filter_size, sigma);
      t = vpTime::measureTimeMs() - t;
      std::cout << "Time to do ViSP Gaussian Blur on grayscale images: " << t << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
      cv::Mat matImg, matImg_blur;
      vpImageConvert::convert(I, matImg);
      t = vpTime::measureTimeMs();
      cv::GaussianBlur(matImg, matImg_blur, cv::Size(gaussian_filter_size, gaussian_filter_size), sigma, 0);
      t = vpTime::measureTimeMs() - t;
      std::cout << "Time to do OpenCV Gaussian Blur on grayscale images: " << t << " ms" << std::endl;

      double threshold = 3.;
      unsigned int margin = 3;
      test = check_results(matImg_blur, I_blur, margin, threshold);
      std::cout << "(I_blur == matImg_blur)? " << test << std::endl;

      if (!test) {
        std::cerr << "Failed Gaussian blur filter on grayscale image!" << std::endl;
        return EXIT_FAILURE;
      }
#endif
    }

    {
      // Test Gaussian blur on color image
      std::cout << "\nTest Gaussian Blur on Klimt color image:" << std::endl;

      vpImage<vpRGBa> I_rgb, I_rgb_blur;
      // Test on real image

      if (opt_ppath.empty()) {
        filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
        vpImageIo::read(I_rgb, filename);
      } else {
        filename = opt_ppath;
        vpImageIo::read(I_rgb, filename);
        printf("Image \"%s\" read successfully\n", filename.c_str());
      }

      unsigned int gaussian_filter_size = 7;
      double sigma = 3;
      t = vpTime::measureTimeMs();
      vpImageFilter::gaussianBlur(I_rgb, I_rgb_blur, gaussian_filter_size, sigma);
      t = vpTime::measureTimeMs() - t;
      std::cout << "Time to do ViSP Gaussian Blur on color images: " << t << " ms" << std::endl;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
      cv::Mat matImg_rgb, matImg_rgb_blur;
      vpImageConvert::convert(I_rgb, matImg_rgb);
      t = vpTime::measureTimeMs();
      cv::GaussianBlur(matImg_rgb, matImg_rgb_blur, cv::Size(gaussian_filter_size, gaussian_filter_size), sigma, 0);
      t = vpTime::measureTimeMs() - t;
      std::cout << "Time to do OpenCV Gaussian Blur on color images: " << t << " ms" << std::endl;

      double threshold = 3.;
      unsigned int margin = 3;
      test = check_results(matImg_rgb_blur, I_rgb_blur, margin, threshold);
      std::cout << "(I_rgb_blur == matImg_rgb_blur)? " << test << std::endl;

      if (!test) {
        std::cerr << "Failed Gaussian blur filter on color image!" << std::endl;
        return EXIT_FAILURE;
      }
#endif
    }

  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\ntestImageFilter is ok." << std::endl;
  return EXIT_SUCCESS;
}
