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
 * Test image resize.
 *
 *****************************************************************************/

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

/*!
  \example testImageResize.cpp

  \brief Test image resize.
*/
// List of allowed command line options
#define GETOPTARGS "cdi:W:H:m:bh"

namespace
{
/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param w : Resize width.
  \param h : Resize height.
  \param m : Resize interpolation method.
 */
void usage(const char *name, const char *badparam, std::string ipath, unsigned int &w, unsigned int &h, int &m)
{
  fprintf(stdout, "\n\
  Test image resize.\n\
  \n\
  SYNOPSIS\n\
    %s [-i <input image path>] [-W <width>] [-H <height>] [-m <method>] [-b] [-c] [-d]\n\
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
    -W <width>                                           %u\n\
       Set the new image width.\n\
  \n\
    -H <height>                                          %u\n\
       Set the new image height.\n\
  \n\
    -m <method>                                          %d\n\
       Set resize interpolation method.\n\
  \n\
    -b                                                     \n\
       Run image resize benchmark.\n\
  \n\
    -c                                   \n\
       Disable mouse click.\n\
  \n\
    -d                                   \n\
       Disable image display.\n\
  \n\
    -h\n\
       Print the help.\n\n", ipath.c_str(), w, h, m);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param w : Resize width.
  \param h : Resize height.
  \param method : Resize interpolation method.
  \param benchmark : Run image resize benchmark.
  \param opt_display : Do not display if set.
  \param opt_click : Do not need click if set.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath, unsigned int &w, unsigned int &h, int &method,
                bool &benchmark, bool &opt_display, bool &opt_click)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'W':
      w = (unsigned int)atoi(optarg_);
      break;
    case 'H':
      h = (unsigned int)atoi(optarg_);
      break;
    case 'm':
      method = atoi(optarg_);
      break;
    case 'b':
      benchmark = true;
      break;
    case 'h':
      usage(argv[0], NULL, ipath, w, h, method);
      return false;
      break;

    case 'c':
      opt_click = false;
      break;
    case 'd':
      opt_display = false;
      break;

    default:
      usage(argv[0], optarg_, ipath, w, h, method);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, w, h, method);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;
    unsigned int width = 101;
    unsigned int height = 207;
    int method = 0;
    bool benchmark = false;
    bool opt_display = true;
    bool opt_click = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, width, height, method, benchmark, opt_display, opt_click) == false) {
      exit(EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
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
      usage(argv[0], NULL, ipath, width, height, method);
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
    for (int m = 0; m < 3; m++) {
      std::cout << "Interpolation method: " << m << std::endl;

      vpImage<unsigned char> Itest(3, 4);
      for (unsigned int cpt = 0; cpt < Itest.getSize(); cpt++) {
        Itest.bitmap[cpt] = cpt;
      }
      vpImage<unsigned char> Itest_resize(Itest.getHeight() * 2, Itest.getWidth() * 2),
          Itest_resize2(Itest.getHeight(), Itest.getWidth());
      vpImageTools::resize(Itest, Itest_resize, (vpImageTools::vpImageInterpolationType)m);
      vpImageTools::resize(Itest_resize, Itest_resize2, (vpImageTools::vpImageInterpolationType)m);
      std::cout << "Itest:\n" << Itest << std::endl;
      std::cout << "Itest_resize:\n" << Itest_resize << std::endl;
      std::cout << "Itest_resize2:\n" << Itest_resize2 << std::endl;
      std::cout << "(Itest ==Itest_resize2)? " << (Itest == Itest_resize2) << std::endl;

      Itest.resize(4, 4);
      for (unsigned int cpt = 0; cpt < Itest.getSize(); cpt++) {
        Itest.bitmap[cpt] = cpt;
      }
      vpImageTools::resize(Itest, Itest_resize, Itest.getWidth() / 2, Itest.getHeight() / 2,
                           (vpImageTools::vpImageInterpolationType)m);
      vpImageTools::resize(Itest_resize, Itest_resize2, Itest.getWidth(), Itest.getHeight(),
                           (vpImageTools::vpImageInterpolationType)m);
      std::cout << "\nItest:\n" << Itest << std::endl;
      std::cout << "Itest_resize:\n" << Itest_resize << std::endl;
      std::cout << "Itest_resize2:\n" << Itest_resize2 << std::endl;
      std::cout << "(Itest ==Itest_resize2)? " << (Itest == Itest_resize2) << std::endl << std::endl;
    }

    // Grayscale image
    vpImage<unsigned char> I; // Input image

    // Read the input grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    std::cout << "Read image: " << filename << std::endl;
    vpImageIo::read(I, filename);

    vpImage<unsigned char> I_resize;
    double t = vpTime::measureTimeMs();
    vpImageTools::resize(I, I_resize, width, height, (vpImageTools::vpImageInterpolationType)method);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to resize from " << I.getWidth() << "x" << I.getHeight() << " to " << width << "x" << height
              << ": " << t << " ms" << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX *d1 = new vpDisplayX, *d2 = new vpDisplayX;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV *d1 = new vpDisplayOpenCV, *d2 = new vpDisplayOpenCV;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK *d1 = new vpDisplayGTK, *d2 = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI *d1 = new vpDisplayGDI, *d2 = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D *d1 = new vpDisplayD3D, *d2 = new vpDisplayD3D;
#else
    std::cerr << "No display available!" << std::endl;
    opt_display = false;
#endif

    if (opt_display) {
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) ||         \
    defined(VISP_HAVE_D3D9)
      d1->init(I, 0, 0, "Grayscale image");
      d2->init(I_resize, (int)I.getWidth() + 80, 0, "Grayscale image resized");
#endif

      vpDisplay::display(I);
      vpDisplay::display(I_resize);
      vpDisplay::displayText(I_resize, 20, 20, "Click to continue.", vpColor::red);
      vpDisplay::flush(I);
      vpDisplay::flush(I_resize);

      if (opt_click) {
        vpDisplay::getClick(I_resize);
      }
    }

    // Color image
    vpImage<vpRGBa> I_color; // Input image

    // Read the input grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    std::cout << "\nRead image: " << filename << std::endl;
    vpImageIo::read(I_color, filename);

    vpImage<vpRGBa> I_color_resize;
    t = vpTime::measureTimeMs();
    vpImageTools::resize(I_color, I_color_resize, width, height, (vpImageTools::vpImageInterpolationType)method);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to resize from " << I_color.getWidth() << "x" << I_color.getHeight() << " to " << width << "x"
              << height << ": " << t << " ms" << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX *d3 = new vpDisplayX, *d4 = new vpDisplayX;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV *d3 = new vpDisplayOpenCV, *d4 = new vpDisplayOpenCV;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK *d3 = new vpDisplayGTK, *d4 = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI *d3 = new vpDisplayGDI, *d4 = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D *d3 = new vpDisplayD3D, *d4 = new vpDisplayD3D;
#else
    std::cerr << "No display available!" << std::endl;
    opt_display = false;
#endif

    if (opt_display) {
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) ||         \
    defined(VISP_HAVE_D3D9)
      d3->init(I_color, 0, 0, "Color image");
      d4->init(I_color_resize, (int)I_color.getWidth() + 80, 0, "Color image resized");
#endif

      vpDisplay::display(I_color);
      vpDisplay::display(I_color_resize);
      vpDisplay::displayText(I_color_resize, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I_color);
      vpDisplay::flush(I_color_resize);
      if (opt_click) {
        vpDisplay::getClick(I_color_resize);
      }
    }

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) ||         \
    defined(VISP_HAVE_D3D9)
    delete d1;
    delete d2;
    delete d3;
    delete d4;
#endif

    vpImage<vpRGBa> I_color_double, I_color_double_half;
    vpImageTools::resize(I_color, I_color_double, I_color.getWidth() * 2, I_color.getHeight() * 2,
                         (vpImageTools::vpImageInterpolationType)method);
    vpImageTools::resize(I_color_double, I_color_double_half, I_color.getWidth(), I_color.getHeight(),
                         (vpImageTools::vpImageInterpolationType)method);
    std::cout << "\n(I_color == I_color_double_half)? " << (I_color == I_color_double_half) << std::endl;

    double root_mean_square_error = 0.0;
    for (unsigned int i = 0; i < I_color.getHeight(); i++) {
      for (unsigned int j = 0; j < I_color.getWidth(); j++) {
        vpColVector c_error = I_color[i][j] - I_color_double_half[i][j];
        root_mean_square_error += c_error.sumSquare();
      }
    }
    std::cout << "Root Mean Square Error: " << sqrt(root_mean_square_error / (I_color.getSize() * 3)) << std::endl;

    vpImage<vpRGBa> I_color_half, I_color_half_double;
    vpImageTools::resize(I_color, I_color_half, I_color.getWidth() / 2, I_color.getHeight() / 2,
                         (vpImageTools::vpImageInterpolationType)method);
    vpImageTools::resize(I_color_half, I_color_half_double, I_color.getWidth(), I_color.getHeight(),
                         (vpImageTools::vpImageInterpolationType)method);
    std::cout << "\n(I_color == I_color_half_double)? " << (I_color == I_color_half_double) << std::endl;

    root_mean_square_error = 0.0;
    for (unsigned int i = 0; i < I_color.getHeight(); i++) {
      for (unsigned int j = 0; j < I_color.getWidth(); j++) {
        vpColVector c_error = I_color[i][j] - I_color_half_double[i][j];
        root_mean_square_error += c_error.sumSquare();
      }
    }
    std::cout << "Root Mean Square Error: " << sqrt(root_mean_square_error / (I_color.getSize() * 3)) << std::endl;

    if (benchmark) {
#if defined(VISP_HAVE_OPENCV) && !defined(__mips__) && !defined(__mips) && !defined(mips) && !defined(__MIPS__)
      std::vector<double> scales;
      scales.push_back(2.0);
      scales.push_back(3.0);
      scales.push_back(4.0);
      scales.push_back(5.0);
      scales.push_back(1 / 2.0);
      scales.push_back(1 / 3.0);
      scales.push_back(1 / 4.0);
      scales.push_back(1 / 5.0);

      std::vector<vpImageTools::vpImageInterpolationType> interpolations;
      interpolations.push_back(vpImageTools::INTERPOLATION_NEAREST);
      interpolations.push_back(vpImageTools::INTERPOLATION_LINEAR);
      interpolations.push_back(vpImageTools::INTERPOLATION_CUBIC);

      std::vector<int> interpolationsCV;
      interpolationsCV.push_back(cv::INTER_NEAREST);
      interpolationsCV.push_back(cv::INTER_LINEAR);
      interpolationsCV.push_back(cv::INTER_CUBIC);

      std::vector<std::string> interpolationNames;
      interpolationNames.push_back("INTERPOLATION_NEAREST");
      interpolationNames.push_back("INTERPOLATION_LINEAR");
      interpolationNames.push_back("INTERPOLATION_CUBIC");
      {
        vpImage<unsigned char> I_resize_perf;
        cv::Mat img, img_resize_perf;
        vpImageConvert::convert(I, img);

        for (size_t i = 0; i < interpolations.size(); i++) {
          std::cout << "\nInterpolation (gray): " << interpolationNames[i] << std::endl;

          for (size_t s = 0; s < scales.size(); s++) {
            unsigned int width_resize = static_cast<unsigned int>(I.getWidth() * scales[s]);
            unsigned int height_resize = static_cast<unsigned int>(I.getHeight() * scales[s]);
            cv::Size new_size(static_cast<int>(width_resize), static_cast<int>(height_resize));
            std::cout << "Resize from " << I.getWidth() << "x" << I.getHeight() << " to "
                      << width_resize << "x" << height_resize << std::endl;

            double t = vpTime::measureTimeMs();
            for (int nbIter = 0; nbIter < 10; nbIter++) {
              vpImageTools::resize(I, I_resize_perf, width_resize, height_resize, interpolations[i]);
            }
            t = vpTime::measureTimeMs() - t;

            double t_cv = vpTime::measureTimeMs();
            for (int nbIter = 0; nbIter < 10; nbIter++) {
              cv::resize(img, img_resize_perf, new_size, 0.0, 0.0, interpolationsCV[i]);
            }
            t_cv = vpTime::measureTimeMs() - t_cv;

            std::cout << "ViSP (10 iterations): " << t << " ms ; Mean: " << t / 10 << " ms" << std::endl;
            std::cout << "OpenCV (10 iterations): " << t_cv << " ms ; Mean: " << t_cv / 10 << " ms" << std::endl;

            double diff = 0.0, diff_abs = 0.0;
            for (int i = 0; i < img_resize_perf.rows; i++) {
              for (int j = 0; j < img_resize_perf.cols; j++) {
                int d = img_resize_perf.at<uchar>(i, j) - I_resize_perf[i][j];
                diff += d;
                diff_abs += vpMath::abs(d);
              }
            }

            std::cout << "Mean diff: " << (diff / I_resize_perf.getSize()) << std::endl;
            std::cout << "Mean abs diff: " << (diff_abs / I_resize_perf.getSize()) << std::endl;
          }
        }
      }

      {
        vpImage<vpRGBa> I_resize_perf;
        cv::Mat img, img_resize_perf;
        vpImageConvert::convert(I_color, img);

        for (size_t i = 0; i < interpolations.size(); i++) {
          std::cout << "\nInterpolation (color): " << interpolationNames[i] << std::endl;

          for (size_t s = 0; s < scales.size(); s++) {
            unsigned int width_resize = static_cast<unsigned int>(I.getWidth() * scales[s]);
            unsigned int height_resize = static_cast<unsigned int>(I.getHeight() * scales[s]);
            cv::Size new_size(static_cast<int>(width_resize), static_cast<int>(height_resize));
            std::cout << "Resize from " << I_color.getWidth() << "x" << I_color.getHeight() << " to "
                      << width_resize << "x" << height_resize << std::endl;

            double t = vpTime::measureTimeMs();
            for (int nbIter = 0; nbIter < 10; nbIter++) {
              vpImageTools::resize(I_color, I_resize_perf, width_resize, height_resize, interpolations[i]);
            }
            t = vpTime::measureTimeMs() - t;

            double t_cv = vpTime::measureTimeMs();
            for (int nbIter = 0; nbIter < 10; nbIter++) {
              cv::resize(img, img_resize_perf, new_size, 0.0, 0.0, interpolationsCV[i]);
            }
            t_cv = vpTime::measureTimeMs() - t_cv;

            std::cout << "ViSP (10 iterations): " << t << " ms ; Mean: " << t / 10 << " ms" << std::endl;
            std::cout << "OpenCV (10 iterations): " << t_cv << " ms ; Mean: " << t_cv / 10 << " ms" << std::endl;

            double diff = 0.0, diff_abs = 0.0;
            for (int i = 0; i < img_resize_perf.rows; i++) {
              for (int j = 0; j < img_resize_perf.cols; j++) {
                int d = (img_resize_perf.at<cv::Vec3b>(i, j)[0] - I_resize_perf[i][j].B) +
                        (img_resize_perf.at<cv::Vec3b>(i, j)[1] - I_resize_perf[i][j].G) +
                        (img_resize_perf.at<cv::Vec3b>(i, j)[2] - I_resize_perf[i][j].R);
                diff += d;
                diff_abs += vpMath::abs(d);
              }
            }

            std::cout << "Mean diff: " << (diff / I_resize_perf.getSize()) << std::endl;
            std::cout << "Mean abs diff: " << (diff_abs / I_resize_perf.getSize()) << std::endl;
          }
        }
      }
#endif
    }

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
